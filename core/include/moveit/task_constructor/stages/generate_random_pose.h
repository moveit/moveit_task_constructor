/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc
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
 *   * Neither the name of PickNik Inc nor the names of its
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

/* Authors: Henning Kayser
   Desc:    Generator Stage for randomized poses based on GeneratePose
*/

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>

#include <random>

namespace moveit {
namespace task_constructor {
namespace stages {

class GenerateRandomPose : public GeneratePose
{
public:
	GenerateRandomPose(const std::string& name = "generate random pose");

	bool canCompute() const override;
	void compute() override;

	/** Function signature for pose dimension samplers.
	 * The input parameter is the seed, the returned value is the sampling result. */
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

	/** Configure a RandomNumberDistribution sampler for a PoseDimension (X/Y/Z/ROLL/PITCH/YAW)
	 *
	 * RandomNumberDistribution is a template class that generates random numbers according to a specific distribution
	 * (see https://en.cppreference.com/w/cpp/numeric/random).
	 * Supported distributions are: std::normal_distribution and std::uniform_real_distribution.
	 * The width parameter specifies the standard deviation resp. the range of the uniform distribution.
	 *
	 * The order in which the PoseDimension samplers are specified matters as the samplers are applied in sequence.
	 * That way it's possible to implement different Euler angles (i.e. XYZ, ZXZ, YXY) or even construct more complex
	 * sampling regions by applying translations after rotations.
	 */
	template <template <class Realtype = double> class RandomNumberDistribution>
	void sampleDimension(const PoseDimension pose_dimension, double width) {
		sampleDimension(pose_dimension, getPoseDimensionSampler<RandomNumberDistribution>(width));
	}

	/** Specify a sampling function of type PoseDimensionSampler for randomizing a pose dimension. */
	void sampleDimension(const PoseDimension pose_dimension, const PoseDimensionSampler& pose_dimension_sampler) {
		pose_dimension_samplers_.emplace_back(std::make_pair(pose_dimension, pose_dimension_sampler));
	}

	/** Limit the number of generated solutions */
	void setMaxSolutions(size_t max_solutions) { setProperty("max_solutions", max_solutions); }

private:
	/** Allocate the sampler function for the specified random distribution */
	template <template <class Realtype = double> class RandomNumberDistribution>
	PoseDimensionSampler getPoseDimensionSampler(double /* width */) {
		static_assert(sizeof(RandomNumberDistribution<double>) == -1, "This distribution type is not supported!");
		throw 0;  // suppress -Wreturn-type
	}

	std::vector<std::pair<PoseDimension, PoseDimensionSampler>> pose_dimension_samplers_;
};
template <>
GenerateRandomPose::PoseDimensionSampler
GenerateRandomPose::getPoseDimensionSampler<std::normal_distribution>(double stddev);
template <>
GenerateRandomPose::PoseDimensionSampler
GenerateRandomPose::getPoseDimensionSampler<std::uniform_real_distribution>(double range);
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
