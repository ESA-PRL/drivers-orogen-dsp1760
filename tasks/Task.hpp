#ifndef DSP1760_TASK_TASK_HPP
#define DSP1760_TASK_TASK_HPP

#include "dsp1760/TaskBase.hpp"
#include <dsp1760/dsp1760.hpp>
#include <aggregator/TimestampEstimator.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <base/samples/imu.h>
#include <fstream>

namespace dsp1760
{
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        dsp1760::DSP1760driver *driver;

        aggregator::TimestampEstimator* timestamp_estimator;
        double sampling_frequency;
        double gyro_integration;

	      base::samples::IMUSensors imu;
	      base::samples::RigidBodyState reading;

        float gyro_bias;
        unsigned long calibration_samples;

    public:
        Task(std::string const& name = "dsp1760::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
	      ~Task();

        void saveBiasValue(float bias);

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
