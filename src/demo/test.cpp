#include <iostream>
#include <memory>

void overloaded( std::unique_ptr<int> const &arg ) {
	std::cout << "  by lvalue " << arg.get() << std::endl;
}
void overloaded( std::unique_ptr<int> && arg ) {
	std::unique_ptr<int> x = std::move(arg);
	std::cout << "  by rvalue, x: " << x.get() << " arg: " << arg.get() << std::endl;
}

/* "t &&" with "t" being template param is special, and  adjusts "t" to be
   (for example) "int &" or non-ref "int" so std::forward knows what to do. */
void forwarding(std::unique_ptr<int> &&arg ) {
    std::cout << "- via std::forward: ";
    overloaded( std::forward<std::unique_ptr<int>&>( arg ) );
    std::cout << "- via std::move: ";
    overloaded( std::move( arg ) ); // conceptually this would invalidate arg
    std::cout << "- by simple passing: ";
    overloaded( arg );
}
void forwarding(std::unique_ptr<int> &arg) {
	std::cout << "* via extra std::move" << std::endl;
	forwarding(std::move(arg));
}

int main() {
    std::cout << "initial caller passes rvalue:\n";
    forwarding(std::make_unique<int>(5));
    std::cout << "initial caller passes lvalue:\n";
    auto x = std::make_unique<int>(5);
    forwarding( x );
}
