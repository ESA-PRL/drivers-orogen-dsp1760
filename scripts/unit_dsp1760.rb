#!/usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'readline'
include Orocos

# Initialize bundles to find the configurations for the packages
Bundles.initialize

# Execute the task
Orocos::Process.run 'unit_dsp1760' do
    # Configure
    gyro = TaskContext.get 'dsp1760'
    gyro.port = "/dev/hdpr-gyro"
    gyro.sampling_frequency = 10
    gyro.calibration_samples = 30
    #gyro.bias = 7.0514832631e-06
    gyro.bias = 0
    #gyro.latitude = 52.2
    gyro.latitude = 0
    gyro.configure

    # Log samples
    logger_gyro = Orocos.name_service.get 'unit_dsp1760_Logger'
    logger_gyro.file = "unit_dsp1760.log"
    logger_gyro.log(gyro.rotation)
    logger_gyro.log(gyro.orientation_samples)
    logger_gyro.log(gyro.bias_samples)
    logger_gyro.log(gyro.bias_values)
    logger_gyro.start

    # Start
    gyro.start
    
    Readline::readline("Press Enter to exit\n") do
    end
end 
