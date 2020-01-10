//
// Copyright 2010-2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <csignal>
#include <iostream>
#include <complex>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t tdd_tx_samps, tdd_rx_samps;
    size_t tx_samps_advance;
    double rate;
    std::string channel_list;
    float ampl;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps_rx", po::value<size_t>(&tdd_rx_samps)->default_value(10000), "total number of samples to receive")
        ("nsamps_tx", po::value<size_t>(&tdd_tx_samps)->default_value(10000), "total number of samples to receive")
        ("tx_samps_advance", po::value<size_t>(&tx_samps_advance)->default_value(10000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of each sample")
        ("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
        return ~0;
    }

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

   //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = std::stoi(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(std::stoi(channel_strings[ch]));
    }

    //set the tx sample rate
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;
    
    //set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    //create a transmit streamer
    uhd::stream_args_t stream_args("fc32", wire); //complex floats
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //allocate buffer with data to send
    std::vector<std::complex<float> > tx_buff(tx_stream->get_max_num_samps(), std::complex<float>(ampl, ampl));

    //create a receive streamer
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //setup metadata for the first tx packet
    uhd::tx_metadata_t tx_md;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst = true;
    tx_md.has_time_spec = true;

    //setup streaming
    std::cout << std::endl;
    std::cout << boost::format(
        "Begin TDD streaming (%u TX %u RX samples), %f seconds in the future..."
    ) % tdd_tx_samps % tdd_rx_samps % seconds_in_future << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = tdd_rx_samps;
    stream_cmd.stream_now = false;

    uhd::time_spec_t timespec_rx = uhd::time_spec_t(seconds_in_future);
    uhd::time_spec_t timespec_tx_tdd = uhd::time_spec_t(0,tdd_tx_samps,rate);
    uhd::time_spec_t timespec_tx_advance = uhd::time_spec_t(0,2*tdd_rx_samps+tdd_tx_samps);
    
    //meta-data will be filled in by recv()
    uhd::rx_metadata_t rx_md;

    //allocate buffer to receive with samples
    std::vector<std::complex<float> > rx_buff(rx_stream->get_max_num_samps());
    std::vector<void *> rx_buffs;
    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
        rx_buffs.push_back(&rx_buff.front()); //same buffer for each channel

    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout (delay before receive + padding)

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to quit..." << std::endl;


    size_t num_acc_rx_samps = 0; //number of accumulated samples
    size_t num_acc_tx_samps = 0; //number of accumulated samples
    do {
        //receive a single packet
        stream_cmd.time_spec = timespec_rx;
        rx_stream->issue_stream_cmd(stream_cmd);

        size_t num_rx_samps = rx_stream->recv(
            rx_buffs, rx_buff.size(), rx_md, timeout, true
        );

        //use a small timeout for subsequent packets
        timeout = 0.1;

        //handle the error code
        if (rx_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
	  std::cerr << "Received timeout..." << std::endl;
	  break;
	}
        if (rx_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % rx_md.strerror()));
        }
	if (num_rx_samps < tdd_rx_samps) {
	  std::cerr << "not all samples transmitted" << std::endl;
	  break;
	}

        if(verbose) std::cout << boost::format(
            "Received packet: %u samples, %u full secs, %f frac secs"
        ) % num_rx_samps % rx_md.time_spec.get_full_secs() % rx_md.time_spec.get_frac_secs() << std::endl;

        //send a single packet 
	tx_md.time_spec = rx_md.time_spec + timespec_tx_advance; 

        size_t num_tx_samps = tx_stream->send(
            &tx_buff.front(), tdd_tx_samps, tx_md, timeout
        );

	if (num_tx_samps < tdd_tx_samps) std::cerr << "Send timeout..." << std::endl;
        if(verbose) std::cout << boost::format("Sent packet: %u samples") % num_tx_samps << std::endl;

	timespec_rx = timespec_rx + timespec_tx_tdd;

    } while (not stop_signal_called);

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
