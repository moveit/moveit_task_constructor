#pragma once

#include <type_traits>
#include <initializer_list>

/** template class to compose flags from enums in a type-safe fashion */
template<typename Enum>
class Flags
{
	static_assert((sizeof(Enum) <= sizeof(int)),
	              "Flags uses an int as storage, this enum will overflow!");
public:
	typedef typename std::conditional<std::is_unsigned<Enum>::value, unsigned int, signed int>::type Int;
	typedef Enum enum_type;
	// compiler-generated copy/move ctor/assignment operators are fine!

	// zero flags
	constexpr inline Flags() noexcept : i(Int(0)) {}
	// initialization from single enum
	constexpr inline Flags(Enum f) noexcept : i(Int(f)) {}
	// initialization from initializer_list
	constexpr inline Flags(std::initializer_list<Enum> flags) noexcept
	   : i(initializer_list_helper(flags.begin(), flags.end())) {}

	const inline Flags &operator&=(int mask) noexcept { i &= mask; return *this; }
	const inline Flags &operator&=(unsigned int mask) noexcept { i &= mask; return *this; }
	const inline Flags &operator&=(Enum mask) noexcept { i &= Int(mask); return *this; }
	const inline Flags &operator|=(Flags f) noexcept { i |= f.i; return *this; }
	const inline Flags &operator|=(Enum f) noexcept { i |= Int(f); return *this; }
	const inline Flags &operator^=(Flags f) noexcept { i ^= f.i; return *this; }
	const inline Flags &operator^=(Enum f) noexcept { i ^= Int(f); return *this; }

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

	constexpr inline bool testFlag(Enum f) const noexcept { return (i & Int(f)) == Int(f) && (Int(f) != 0 || i == Int(f) ); }

private:
	constexpr inline Flags(Int i) : i(i) {}
	constexpr static inline Int initializer_list_helper(typename std::initializer_list<Enum>::const_iterator it,
	                                                    typename std::initializer_list<Enum>::const_iterator end) noexcept {
		return (it == end ? Int(0) : (Int(*it) | initializer_list_helper(it + 1, end)));
	}

	Int i;
};

#define DECLARE_FLAGS(Flags, Enum) typedef QFlags<Enum> Flags;
