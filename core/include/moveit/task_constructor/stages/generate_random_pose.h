/* Authors: Henning Kayser
   Desc:    Generator Stage for random poses based on GeneratePose
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
	 * * std::uniform_real_distrubtion: distribution_param specifies the full value range around the target_pose value
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
	 * Limits the number of generated solutions
	 */
	void setMaxSolutions(size_t max_solutions) { setProperty("max_solutions", max_solutions); }

private:
	/* \brief Allocate the sampler function for the specified random distribution */
	template <template <class Realtype = double> class RandomNumberDistribution>
	PoseDimensionSampler getPoseDimensionSampler(double /* distribution_param */) {
		static_assert(sizeof(RandomNumberDistribution<double>) == -1, "This distribution type is not supported!");
		throw 0;  // suppress -Wreturn-type
	}

	std::map<PoseDimension, PoseDimensionSampler> pose_dimension_samplers_;
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
