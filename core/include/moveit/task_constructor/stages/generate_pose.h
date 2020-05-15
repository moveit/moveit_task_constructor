/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner
   Desc:    Generator Stage for poses
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/cost_queue.h>
#include <geometry_msgs/PoseStamped.h>

#include <random>

namespace moveit {
namespace task_constructor {
namespace stages {

class GeneratePose : public MonitoringGenerator
{
public:
	GeneratePose(const std::string& name = "generate pose");

	void reset() override;
	bool canCompute() const override;
	void compute() override;

	void setPose(const geometry_msgs::PoseStamped& pose) { setProperty("pose", pose); }

	/*
	 * Function specification for pose dimension samplers.
	 * The input paramter is the target_pose value used as seed, the returned value is the sampling result.
	 * */
	typedef std::function<double(double)> PoseDimensionSampler;
	enum PoseDimension
	{
		X,
		Y,
		Z,
		ROLL,
		PITCH,
		YAW
	};

	/*
	 * Configure a RandomNumberDistribution (see https://en.cppreference.com/w/cpp/numeric/random) sampler for a
	 * PoseDimension (X/Y/Z/ROLL/PITCH/YAW) for randomizing the pose.
	 * The distribution_param is applied to the specified distribution method while the target pose value is used as
	 * seed.
	 * Currently supported distributions are:
	 * * std::uniform_real_distrubtion: distribution_param specifies the +- value range around the target_pose value
	 * * std::normal_distribution: distribution_param is used as stddev, target_pose value is mean
	 *
	 * Other distributions can be used by setting the PoseDimensionSampler directly using the function overload.
	 */
	template <template <class Realtype = double> class RandomNumberDistribution>
	void sampleDimension(const PoseDimension pose_dimension, double distribution_param) {
		sampleDimension(pose_dimension, getPoseDimensionSampler<RandomNumberDistribution>(distribution_param));
	}

	/*
	 * Specify a sampling function of type PoseDimensionSampler for randomizing a pose dimension.
	 */
	void sampleDimension(const PoseDimension pose_dimension, const PoseDimensionSampler& pose_dimension_sampler) {
		pose_dimension_samplers_[pose_dimension] = pose_dimension_sampler;
	}

	/*
	 * Limits the number of spawned solution in case randomized sampling is enabled using `sampleDimension()`.
	 */
	void setMaxSolutions(size_t max_solutions) { setProperty("max_solutions", max_solutions); }

protected:
	void onNewSolution(const SolutionBase& s) override;
	ordered<const SolutionBase*> upstream_solutions_;

private:
	/* \brief Allocate the sampler function for the specified random distribution */
	template <template <class Realtype = double> class RandomNumberDistribution>
	PoseDimensionSampler getPoseDimensionSampler(double distribution_param) {
		static_assert(sizeof(RandomNumberDistribution<double>) == -1, "This distribution type is not supported!");
	}

	std::map<PoseDimension, PoseDimensionSampler> pose_dimension_samplers_;
};
template <>
GeneratePose::PoseDimensionSampler
GeneratePose::getPoseDimensionSampler<std::normal_distribution>(double distribution_param);
template <>
GeneratePose::PoseDimensionSampler
GeneratePose::getPoseDimensionSampler<std::uniform_real_distribution>(double distribution_param);
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
