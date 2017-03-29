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

    // Configuration of time estimator
    timestamp_estimator = new aggregator::TimestampEstimator(
	    base::Time::fromSeconds(20),
	    base::Time::fromSeconds(1.0 / sampling_frequency),
	    base::Time::fromSeconds(0),
	    INT_MAX);

    // Initial offset is set to 0
    calibrating = false;
    gyro_bias = _gyro_bias.value();
    calibration_samples = 0;

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
    if(!driver->update(rotation_delta))
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
    imu.gyro[2] = rotation_delta - gyro_bias;

    _rotation.write(imu);

    // Write out the integrated output
    reading.time = timestamp_estimated;

    gyro_integration += rotation_delta;
    reading.orientation = Eigen::AngleAxisd(gyro_integration, Eigen::Vector3d::Unit(2));
    reading.angular_velocity = Eigen::Vector3d(0, 0, rotation_delta);
    _orientation_samples.write(reading);

    if(calibrating)
    {
        if(calibration_samples >= _calibration_samples.value())
        {
            // Save the bias value in the properties of the gyro
            _gyro_bias.value() = gyro_bias;
            //saveBiasValue(gyro_bias);
            calibration_samples = 0;
            calibrating = false;
        }
        else
        {
            // Do a rolling average of the rotation value
            gyro_bias = (gyro_bias * calibration_samples + rotation_delta) / (calibration_samples + 1);
            calibration_samples++;
        }
    }

    // # Throws some message in rock-display saying that unknown_t is not defined...
    //_timestamp_estimator_status.write(timestamp_estimator->getStatus());
}

void Task::saveBiasValue(float bias)
{
    // Save the bias calibration values
    char filename[240];
    // TODO ARGH! NO! Don't do this!!!! Also remove the include form the header
    sprintf(filename, "~/dev/bundles/hdpr/logs/current/dsp1760_estimated_bias.txt");
    std::ofstream file;
    file.open(filename);
    file << "******** DSP1760 Gyroscope Bias Offset *******"<<"\n";
    file << "Z-axis gyroscope bias offset: " << bias <<"\n";
    file.close();
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
