#include "Task.hpp"

using namespace dsp1760;

Task::Task(std::string const& name):
    TaskBase(name)
{
    timestamp_estimator = NULL;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine)
{
    timestamp_estimator = NULL;
}

Task::~Task()
{
    if(timestamp_estimator != NULL)
    {
        delete timestamp_estimator;
        timestamp_estimator = NULL;
    }
}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }

    // Instantiate the driver
    driver = new dsp1760::DSP1760driver();

    if(!driver->openSerial(_port.value(), _baudrate.value()))
    {
        fprintf(stderr, "DSP1760: Cannot initialize driver\n");
        return false;
    }

    // Get the sensor sampling frequency for the timestamp estimator
    sampling_frequency = _sampling_frequency.value();

    driver->setDataRate(sampling_frequency);
    // Set the temperature reading to output decimals
    driver->setTemperatureDecimal(true);
    // Make sure the gyro outputs the rate (by default set to delta)
    driver->setAngularDataFormat(RATE);

    // Configuration of time estimator
    timestamp_estimator = new aggregator::TimestampEstimator(
	    base::Time::fromSeconds(20),
	    base::Time::fromSeconds(1.0 / sampling_frequency),
	    base::Time::fromSeconds(0),
	    INT_MAX);

    // Initial offset is set to 0
    bias = _bias.value();
    calibration_samples = 0;
    latitude_rad = _latitude.value() * M_PI / 180.0f;

    // Inisialise the integrated gyro value (absolute heading)
    gyro_integration = _initial_heading.value();

    // Define all IMU output fields to 0
    imu.gyro[0] = 0.0f;
    imu.gyro[1] = 0.0f;
    imu.gyro[2] = 0.0f;
    imu.acc[0] = 0.0f;
    imu.acc[1] = 0.0f;
    imu.acc[2] = 0.0f;
    imu.mag[0] = 0.0f;
    imu.mag[1] = 0.0f;
    imu.mag[2] = 0.0f;

    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }

    // Mandatory for a FD driven component
    RTT::extras::FileDescriptorActivity* activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if(activity)
    {
        activity->watch(driver->getFileDescriptor());
        // Set the timeout to 2 seconds, slowest sampling time is 1Hz, this gives a 1 second margin
        activity->setTimeout(2000);
    }
    else
    {
        fprintf(stderr, "DSP1760: File descriptor error\n");
        return false;
    }

    // Initialise the last_time to 0
    last_time = base::Time();
    timestamp_estimator->reset();

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    RTT::extras::FileDescriptorActivity* activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if(activity)
    {
        if(activity->hasError())
        {
            fprintf(stderr, "DSP1760: IO error\n");
        }

        if(activity->hasTimeout())
        {
            fprintf(stderr, "DSP1760: Timeout\n");
        }
    }
    else
    {
        fprintf(stderr, "DSP1760: No RTT activity\n");
        return;
    }

    float rotation_delta;
    float temperature;
    if(!driver->update(rotation_delta, temperature, sequence_counter))
    {
        fprintf(stderr, "DSP1760: Error reading gyroscope\n");
    }

    // Get the current timestamp and guestimate the best sample time
    // http://rock-robotics.org/stable/documentation/data_processing/timestamping.html
    base::Time timestamp = base::Time::now();
    base::Time timestamp_estimated = timestamp_estimator->update(timestamp, driver->getIndex());

    // Update the IMU timestamp
    imu.time = timestamp_estimated;
    // Update the Z rotation value
    imu.gyro[2] = rotation_delta;

    // Write out the raw rotation value
    _rotation_raw.write(imu);

    // Remove the earth rotation component
    // TODO: figure out if the Earth rotation must be added or removed (depends on hemisphere)
    float earth_rotation_bias = EARTH_ROTATION_RATE * sin(latitude_rad);
    imu.gyro[2] -= earth_rotation_bias;

    // Run the bias calibration process
    if(_calibrate.value())
    {
        if(state() != CALIBRATING)
        {
            state(CALIBRATING);
        }

        // Calibration samples acquired
        if(calibration_samples >= (unsigned)_calibration_samples.value())
        {
            // Save the bias value in the properties of the component
            _bias.value() = bias;
            // Output the bias value to the console for easy access
            printf("DSP1760 bias value: %.*e\n", 10, bias);

            // Output to a port for logger to save the bias value
            dsp1760::samples::Bias bias_sample;
            bias_sample.time = base::Time::now();
            bias_sample.bias_value = bias;
            _bias_values.write(bias_sample);

            // Reset calibration values
            calibration_samples = 0;
            _calibrate.value() = false;
        }
        else
        {
            // Do a rolling average of the rotation value
            bias = (bias * calibration_samples + imu.gyro[2]) / (calibration_samples + 1);
            // Bias sample values are already compensated for Earth rotation bias
            _bias_samples.write(imu);
            calibration_samples++;
        }
    }
    else if(state() != RUNNING)
    {
        state(RUNNING);
    }

    // Remove the static bias
    imu.gyro[2] -= bias;

    _rotation.write(imu);

    // Publish the sensor temperature as well
    _temperature.write(temperature);

    // Write out the integrated output (outputs the absolute yaw orientation in a quaternion)
    reading.time = timestamp_estimated;

    // First value of last_time is initialised to current time
    if(last_time == base::Time())
    {
        last_time = reading.time;
    }
    // Integrate the gyro to get the yaw position
    base::Time delta_time = reading.time - last_time;
    last_time = reading.time;

    gyro_integration += imu.gyro[2] * delta_time.toSeconds();
    reading.orientation = Eigen::AngleAxisd(gyro_integration, Eigen::Vector3d::Unit(2));
    // Also set the angular velocity since there is a field for that
    reading.angular_velocity = Eigen::Vector3d(0, 0, imu.gyro[2]);
    _orientation_samples.write(reading);
    
    _sequence_counter.write(sequence_counter);

    // # Throws some message in rock-display saying that unknown_t is not defined...
    //_timestamp_estimator_status.write(timestamp_estimator->getStatus());
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // Clear the FD watch
    RTT::extras::FileDescriptorActivity* activity = getActivity<RTT::extras::FileDescriptorActivity>();
    if(activity)
    {
        activity->clearAllWatches();
    }

    driver->close();
    timestamp_estimator->reset();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    driver->close();
    delete driver;

    delete timestamp_estimator;
    timestamp_estimator = NULL;
}
