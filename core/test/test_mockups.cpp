#include "stage_mockups.h"
#include <moveit/task_constructor/container_p.h>

using namespace moveit::task_constructor;

struct TestGeneratorMockup : public GeneratorMockup
{
	InterfacePtr next_starts_;
	InterfacePtr prev_ends_;

	using GeneratorMockup::GeneratorMockup;
	using GeneratorMockup::init;

	void init() {
		next_starts_ = std::make_shared<Interface>();
		prev_ends_ = std::make_shared<Interface>();

		GeneratorMockup::reset();
		GeneratorMockup::init(getModel());

		GeneratorPrivate* impl = pimpl();
		impl->setNextStarts(next_starts_);
		impl->setPrevEnds(prev_ends_);
	}
};

TEST(GeneratorMockup, compute) {
	TestGeneratorMockup gen({ 1.0, 2.0, 3.0 });
	gen.init();

	for (int i = 0; i < 3; ++i) {
		ASSERT_TRUE(gen.canCompute());
		gen.compute();
	}
	EXPECT_FALSE(gen.canCompute());

	EXPECT_COSTS(gen.solutions(), ::testing::ElementsAre(1, 2, 3));
}

#define COMPUTE_EXPECT_COSTS(stage, ...)                                                 \
	{                                                                                     \
		EXPECT_TRUE(stage.canCompute());                                                   \
		stage.compute();                                                                   \
		EXPECT_COSTS(stage.solutions(), ::testing::ElementsAre(__VA_ARGS__));              \
		constexpr auto num_elements = std::initializer_list<double>{ __VA_ARGS__ }.size(); \
		EXPECT_EQ(runs, num_elements);                                                     \
	}

TEST(GeneratorMockup, delayed) {
	auto gen = std::make_unique<TestGeneratorMockup>(PredefinedCosts({ 1.0, 2.0, 3.0 }));
	gen->init();
	auto& runs = gen->runs_;

	DelayingWrapper w({ 1, 0, 2 }, std::move(gen));
	auto prev_ends = std::make_shared<Interface>();
	auto next_starts = std::make_shared<Interface>();

	WrapperBasePrivate* impl = w.pimpl();
	impl->setPrevEnds(prev_ends);
	impl->setNextStarts(next_starts);

	// 1st compute() is delayed by 1
	COMPUTE_EXPECT_COSTS(w);
	COMPUTE_EXPECT_COSTS(w, 1);

	// 2nd compute() is not delayed
	COMPUTE_EXPECT_COSTS(w, 1, 2);

	// 1st compute() is delayed by 2
	COMPUTE_EXPECT_COSTS(w, 1, 2);
	COMPUTE_EXPECT_COSTS(w, 1, 2);
	COMPUTE_EXPECT_COSTS(w, 1, 2, 3);

	EXPECT_FALSE(w.canCompute());
}
#undef COMPUTE_EXPECT_COSTS
