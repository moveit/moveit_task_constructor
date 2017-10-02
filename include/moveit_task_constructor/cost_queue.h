#pragma once

#include <set>
#include <iostream>

/**
 *  @brief cost_ordered<T> provides an adapter for a container (T) to allow for cost ordering
 *
 *  The data container must not invalidate iterators upon insertion at end, i.e. std::vector is not viable!
 *  Use std::list or std::deque instead.
 *
 *  @ingroup sequences
 *
 *  @tparam T        container type
 */
template <typename ContainerType, typename CostType = double, typename Compare = std::less<CostType>>
class cost_ordered
{
public:
	typedef CostType cost_type;
	typedef ContainerType container_type;
	typedef typename container_type::value_type value_type;
	typedef typename container_type::size_type size_type;
	typedef typename container_type::reference reference;
	typedef typename container_type::const_reference const_reference;
	typedef typename container_type::iterator container_iterator;

protected:
	struct cost_item_pair : std::pair<CostType, container_iterator> {
		/// implicit constructor from std::pair
		cost_item_pair(std::pair<CostType, container_iterator>&& other)
		   : std::pair<CostType, container_iterator>(std::move(other))
		{}

		inline const CostType& cost() const { return this->first; }
		inline reference value() { return *this->second; }
		inline const_reference value() const { return *this->second; }

		// comparison only considers costs
		constexpr bool operator<(const cost_item_pair& other) {
			return Compare(this->cost(), other.cost());
		}
		// define consistent greater
		constexpr bool operator>(const cost_item_pair& other) {
			return !operator<(other);
		}
	};

public:
	typedef std::multiset<cost_item_pair, std::less<cost_item_pair>> sorted_type;
	typedef typename sorted_type::iterator sorted_iterator;
	typedef typename sorted_type::const_iterator sorted_const_iterator;

protected:
	container_type const & container_; // contains the actual data items
	sorted_type sorted_; // sorted queue

public:
	/// initialize empty list
	explicit cost_ordered(ContainerType& c) : container_(c) {}

	bool empty() const { return container_.empty(); }
	size_type size() const { return container_.size(); }

	const sorted_type& sorted() const { return sorted_; }

	void clear() { sorted_.clear(); }

	cost_item_pair top() const { return *sorted_.begin(); }

	/// push container iterator to sorted queue
	sorted_iterator push_back(const container_iterator& item, const CostType& cost = CostType()) {
		return sorted_.insert(std::make_pair(cost, item));
	}

	/// update costs of given container_iterator
	sorted_iterator set_cost(const sorted_iterator &it, CostType &&cost = CostType()) {
		container_iterator item = it->second;
		sorted_.erase(it);
		return sorted_.insert(std::make_pair(std::move(cost), item));
	}
};

#if 0 && __cplusplus >= 201103L
template<typename ContainerType>
constexpr cost_ordered<typename ContainerType, typename std::greater<double>>
make_cost_ordered(ContainerType& __c)
{
	// TODO use std::forward + reference stripping from std::make_pair
	return cost_ordered<ContainerType, double>(__c);
}
#else
template<class ContainerType>
inline cost_ordered<ContainerType>
make_cost_ordered(ContainerType& __c)
{
	return cost_ordered<ContainerType, double>(__c);
}
#endif
