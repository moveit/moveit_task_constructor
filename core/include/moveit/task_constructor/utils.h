/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Authors: Robert Haschke, Michael 'v4hn' Goerner
   Desc:    Miscellaneous utilities
*/

#pragma once

#include <string>
#include <type_traits>
#include <initializer_list>

#include <Eigen/Geometry>

#include <moveit/macros/class_forward.h>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace moveit {

namespace core {
MOVEIT_CLASS_FORWARD(LinkModel);
MOVEIT_CLASS_FORWARD(JointModelGroup);
MOVEIT_CLASS_FORWARD(RobotState);
}  // namespace core

namespace task_constructor {
MOVEIT_CLASS_FORWARD(Property);

namespace utils {

/** template class to compose flags from enums in a type-safe fashion */
template <typename Enum>
class Flags
{
	static_assert((sizeof(Enum) <= sizeof(int)), "Flags uses an int as storage, this enum will overflow!");

public:
	using Int = typename std::conditional<std::is_unsigned<Enum>::value, unsigned int, int>::type;
	using enum_type = Enum;
	// compiler-generated copy/move ctor/assignment operators are fine!

	// zero flags
	constexpr inline Flags() noexcept : i(Int(0)) {}
	// initialization from single enum
	constexpr inline Flags(Enum f) noexcept : i(Int(f)) {}
	// initialization from initializer_list
	constexpr inline Flags(std::initializer_list<Enum> flags) noexcept
	  : i(initializer_list_helper(flags.begin(), flags.end())) {}

	const inline Flags& operator&=(int mask) noexcept {
		i &= mask;
		return *this;
	}
	const inline Flags& operator&=(unsigned int mask) noexcept {
		i &= mask;
		return *this;
	}
	const inline Flags& operator&=(Enum mask) noexcept {
		i &= Int(mask);
		return *this;
	}
	const inline Flags& operator|=(Flags f) noexcept {
		i |= f.i;
		return *this;
	}
	const inline Flags& operator|=(Enum f) noexcept {
		i |= Int(f);
		return *this;
	}
	const inline Flags& operator^=(Flags f) noexcept {
		i ^= f.i;
		return *this;
	}
	const inline Flags& operator^=(Enum f) noexcept {
		i ^= Int(f);
		return *this;
	}

	constexpr inline operator Int() const noexcept { return i; }

	constexpr inline Flags operator|(Flags f) const noexcept { return Flags(i | f.i); }
	constexpr inline Flags operator|(Enum f) const noexcept { return Flags(i | Int(f)); }
	constexpr inline Flags operator^(Flags f) const noexcept { return Flags(i ^ f.i); }
	constexpr inline Flags operator^(Enum f) const noexcept { return Flags(i ^ Int(f)); }
	constexpr inline Flags operator&(int mask) const noexcept { return Flags(i & mask); }
	constexpr inline Flags operator&(unsigned int mask) const noexcept { return Flags(i & mask); }
	constexpr inline Flags operator&(Enum f) const noexcept { return Flags(i & Int(f)); }
	constexpr inline Flags operator~() const noexcept { return Flags(~i); }

	constexpr inline bool operator!() const noexcept { return !i; }

	constexpr inline bool testFlag(Enum f) const noexcept {
		return (i & Int(f)) == Int(f) && (Int(f) != 0 || i == Int(f));
	}

private:
	constexpr inline Flags(Int i) : i(i) {}
	constexpr static inline Int
	initializer_list_helper(typename std::initializer_list<Enum>::const_iterator it,
	                        typename std::initializer_list<Enum>::const_iterator end) noexcept {
		return (it == end ? Int(0) : (Int(*it) | initializer_list_helper(it + 1, end)));
	}

	Int i;
};

bool getRobotTipForFrame(const Property& property, const planning_scene::PlanningScene& scene,
                         const moveit::core::JointModelGroup* jmg, std::string& error_msg,
                         const moveit::core::LinkModel*& robot_link, Eigen::Isometry3d& tip_in_global_frame);
}  // namespace utils
}  // namespace task_constructor
}  // namespace moveit
