#pragma once

#include <set>
#include <deque>
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
template <typename ValueType,
          typename ContainerType = std::deque<ValueType>,
          typename CostType = double,
          typename Compare = std::less<CostType>>
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
	container_type managed_container_; // item container, owned by this instance
	container_type* container_; // reference to the actually used item container
	sorted_type sorted_; // sorted queue

public:
	/// initialize empty self-managed container
	explicit cost_ordered()
	   : managed_container_()
	   , container_(&managed_container_) {}
	/// initialize from externally managed container c
	explicit cost_ordered(ContainerType& c)
	   : container_(&c) {}

#if 0 // TODO: not yet working
	/// initialize from externally managed container, but take ownership
	explicit cost_ordered(ContainerType&& c)
	   : managed_container_(std::move<ContainerType>(c.managed_container_))
	   , container_(&managed_container_){}
#endif

	bool empty() const { return container_->empty(); }
	size_type size() const { return container_->size(); }

	const container_type& items() const { return *container_; }
	const sorted_type& sorted() const { return sorted_; }

	void clear() { sorted_.clear(); }

	cost_item_pair top() const { return *sorted_.begin(); }

	/// push new value to container_ + sorted_
	sorted_iterator push_back(const value_type& value, const CostType& cost = CostType()) {
		auto item = container_->insert(container_->cend(), value);
		return sorted_.insert(std::make_pair(cost, item));
	}

	/// update costs of given container_iterator
	sorted_iterator set_cost(const sorted_iterator &it, CostType &&cost = CostType()) {
		container_iterator item = it->second;
		sorted_.erase(it);
		return sorted_.insert(std::make_pair(std::move(cost), item));
	}
};

#if 0 // TODO: allow make_cost_ordered<CostType>(ContainerType)
template<typename ContainerType, typename CostType = double, typename Compare = std::less<CostType>>
auto make_cost_ordered(const ContainerType &c) {
	return cost_ordered<typename ContainerType::value_type, ContainerType, CostType, Compare>(c);
}
#endif
