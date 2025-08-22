/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Authors: Joseph Moore */

#include <moveit/task_constructor/stages/limit_solutions.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/cost_terms.h>
#include <fmt/format.h>

namespace moveit {
namespace task_constructor {
namespace stages {

LimitSolutions::LimitSolutions(const std::string& name, Stage::pointer&& child) : WrapperBase(name, std::move(child)) {
	auto& p = properties();
	p.declare<uint32_t>("max_solutions", "maximum number of solutions returned by this wrapper");
	forwarded_solutions = 0;
}

void LimitSolutions::reset() {
	upstream_solutions_.clear();
	forwarded_solutions = 0;
	WrapperBase::reset();
}

void LimitSolutions::onNewSolution(const SolutionBase& s) {
	uint32_t max_solutions = properties().get<uint32_t>("max_solutions");
	if (forwarded_solutions + upstream_solutions_.size() < max_solutions)
		upstream_solutions_.push(&s);
}

bool LimitSolutions::canCompute() const {
	return !upstream_solutions_.empty() || WrapperBase::canCompute();
}

void LimitSolutions::compute() {
	if (WrapperBase::canCompute())
		WrapperBase::compute();

	if (upstream_solutions_.empty())
		return;

	++forwarded_solutions;
	liftSolution(*upstream_solutions_.pop());
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
