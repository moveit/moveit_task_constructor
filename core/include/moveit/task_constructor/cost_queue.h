#pragma once

#include <queue>
#include <list>
#include <deque>
#include <iostream>
#include <algorithm>

/// ValueOrPointeeLess provides correct comparison for plain and pointer-like types
template <typename T, typename = bool>
struct ValueOrPointeeLess : public std::less<T>
{};

/// The following template-specialization is for pointer-like types
template <typename T>
struct ValueOrPointeeLess<T, decltype(*std::declval<T>() < *std::declval<T>())>
{
	bool operator()(const T& x, const T& y) const { return *x < *y; }
};

/**
 *  @brief ordered<ValueType> provides an adapter for a std::list to allow sorting.
 *
 *  In contrast to std::priority_queue, we use a std::list as the underlying container.
 *  This ensures, that existing iterators remain valid upon insertion and deletion.
 *  Sorted insertion has logarithmic complexity.
 */
template <typename T, typename Compare = ValueOrPointeeLess<T>>
class ordered
{
public:
	using container_type = std::list<T>;
	using value_type = typename container_type::value_type;
	using size_type = typename container_type::size_type;
	using difference_type = typename container_type::difference_type;

	using reference = typename container_type::reference;
	using const_reference = typename container_type::const_reference;

	using pointer = typename container_type::pointer;
	using const_pointer = typename container_type::const_pointer;

	using iterator = typename container_type::iterator;
	using const_iterator = typename container_type::const_iterator;

	using reverse_iterator = typename container_type::reverse_iterator;
	using const_reverse_iterator = typename container_type::const_reverse_iterator;

protected:
	container_type c;
	Compare comp;

public:
	/// initialize empty container
	explicit ordered() {}

	bool empty() const { return c.empty(); }
	size_type size() const { return c.size(); }

	void clear() { c.clear(); }

	reference top() { return c.front(); }
	const_reference top() const { return c.front(); }
	value_type pop() {
		value_type result(top());
		c.pop_front();
		return result;
	}

	reference front() { return c.front(); }
	const_reference front() const { return c.front(); }
	reference back() { return c.back(); }
	const_reference back() const { return c.back(); }

	iterator begin() { return c.begin(); }
	iterator end() { return c.end(); }

	const_iterator begin() const { return c.begin(); }
	const_iterator end() const { return c.end(); }
	const_iterator cbegin() const { return c.begin(); }
	const_iterator cend() const { return c.end(); }

	const_reverse_iterator rbegin() const { return c.rbegin(); }
	const_reverse_iterator rend() const { return c.rend(); }
	const_reverse_iterator crbegin() const { return c.rbegin(); }
	const_reverse_iterator crend() const { return c.rend(); }

	/// explicitly sort container, useful if many items have changed their value
	void sort() { c.sort(comp); }

	iterator insert(const value_type& item) {
		iterator at = std::upper_bound(c.begin(), c.end(), item, comp);
		return c.insert(at, item);
	}
	iterator insert(value_type&& item) {
		iterator at = std::upper_bound(c.begin(), c.end(), item, comp);
		return c.insert(at, std::move(item));
	}
	inline void push(const value_type& item) { insert(item); }
	inline void push(value_type&& item) { insert(std::move(item)); }

	iterator erase(const_iterator pos) { return c.erase(pos); }

	/// update sort position of a single item after changes
	iterator update(iterator& it) {
		container_type temp;
		temp.splice(temp.end(), c, it);  // move it from c to temp
		iterator at = std::upper_bound(c.begin(), c.end(), *it, comp);
		c.splice(at, temp, it);
		return it;
	}

	/// move element pos from this to other container, inserting before other_pos
	iterator moveTo(iterator pos, container_type& other, iterator other_pos) {
		other.splice(other_pos, c, pos);
		return pos;
	}
	/// move element pos from other container into this one (sorted)
	iterator moveFrom(iterator pos, container_type& other) {
		iterator at = std::upper_bound(begin(), end(), *pos, comp);
		c.splice(at, other, pos);
		return pos;
	}

	template <typename Predicate>
	void remove_if(Predicate p) {
		c.remove_if(p);
	}
};

namespace detail {

template <typename ValueType, typename CostType>
struct ItemCostPair : std::pair<ValueType, CostType>
{
	using cost_type = CostType;

	ItemCostPair(const std::pair<ValueType, CostType>& other) : std::pair<ValueType, CostType>(other) {}
	ItemCostPair(std::pair<ValueType, CostType>&& other) : std::pair<ValueType, CostType>(std::move(other)) {}

	inline ValueType& value() { return this->first; }
	inline const ValueType& value() const { return this->first; }

	inline CostType cost() const { return this->second; }

	// comparison only considers cost
	constexpr bool operator<(const ItemCostPair& other) const { return this->cost() < other.cost(); }
};

}  // namespace detail

template <typename ValueType, typename CostType = double,
          typename Compare = std::less<detail::ItemCostPair<ValueType, CostType>>>
class cost_ordered : public ordered<detail::ItemCostPair<ValueType, CostType>, Compare>
{
	using base_type = ordered<detail::ItemCostPair<ValueType, CostType>, Compare>;

public:
	auto insert(const ValueType& value, const CostType cost) { return base_type::insert(std::make_pair(value, cost)); }
	auto insert(ValueType&& value, const CostType cost) {
		return base_type::insert(std::make_pair(std::move(value), cost));
	}
};
