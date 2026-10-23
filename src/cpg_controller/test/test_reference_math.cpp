#include <gtest/gtest.h>
#include <limits>
#include "cpg_controller/reference_math.hpp"
#include "cpg_controller/learning_interface.hpp"
using namespace cpg_controller;

TEST(Reference, HarmonicsIncludeRadialAndThirdOrderDerivative) {
  const double t=.37, eps=1e-6;
  auto at=[](double v) { return harmonics(.8+v*.2, -.3+v*.7,.2,.7); };
  const auto a=at(t), p=at(t+eps), m=at(t-eps);
  for (size_t n=1;n<=3;++n) {
    EXPECT_NEAR(a.x[n].rate,(p.x[n].value-m.x[n].value)/(2*eps),1e-9);
    EXPECT_NEAR(a.y[n].rate,(p.y[n].value-m.y[n].value)/(2*eps),1e-9);
  }
  EXPECT_NEAR(a.x[3].value,std::pow(a.x[1].value,3)-3*a.x[1].value*std::pow(a.y[1].value,2),1e-12);
}

TEST(Reference, FullDerivativeDuringModeAndAmplitudeTransition) {
  auto at=[](double t) {
    const auto s=progress(t,1.2);
    Coefficients w;
    w.offset=blend({-.3,0},{.4,0},s);
    const Sample scale{1+.2*t,.2}, envelope{.5+.1*t,.1};
    for (int n=1;n<=3;++n) {
      w.x[n]=blend({.1*n,0},{-.2*n,0},s)*scale;
      w.y[n]=blend({-.3*n,0},{.05*n,0},s)*scale;
    }
    return envelope*readout(w,harmonics(std::cos(2*t),std::sin(2*t),-2*std::sin(2*t),2*std::cos(2*t)));
  };
  for (double t : {.05,.3,.7,1.1})
    EXPECT_NEAR(at(t).rate,(at(t+1e-6).value-at(t-1e-6).value)/2e-6,1e-8);
}

TEST(Reference, ConstraintsUseBoundaryVelocity) {
  EXPECT_DOUBLE_EQ(constrain({2,10},{-1,0},{1,0}).rate,0);
  const auto q=constrain({-2,-10},{-.5,.3},{1,0});
  EXPECT_DOUBLE_EQ(q.value,-.5);
  EXPECT_DOUBLE_EQ(q.rate,.3);
  EXPECT_DOUBLE_EQ(constrain({.5,.7},{-1,0},{1,0}).rate,.7);
}

TEST(Transition, EndpointAndRetargetContinuity) {
  Transition curve;
  curve.set(1,10,.5);
  EXPECT_DOUBLE_EQ(curve.sample(10).value,0);
  EXPECT_DOUBLE_EQ(curve.sample(10).rate,0);
  const auto before=curve.sample(10.2);
  const auto acc=curve.acceleration(10.2);
  curve.set(0,10.2,.5);
  EXPECT_NEAR(curve.sample(10.2).value,before.value,1e-12);
  EXPECT_NEAR(curve.sample(10.2).rate,before.rate,1e-12);
  EXPECT_NEAR(curve.acceleration(10.2),acc,1e-12);
  curve.set(0,10.4,.5);  // must not postpone stopping
  EXPECT_DOUBLE_EQ(curve.sample(10.71).value,0);
  EXPECT_DOUBLE_EQ(curve.sample(10.71).rate,0);
  EXPECT_DOUBLE_EQ(curve.acceleration(10.71),0);
}

TEST(Measurement, RejectStaleFutureDuplicatesAndMaskMissingJoints) {
  MeasurementInput input;
  const std::vector<std::string> names={"lh","lk","la","rh","rk","ra"};
  EXPECT_FALSE(input.accept({"lh"},{1},0,1,.1,names));
  EXPECT_FALSE(input.accept({"lh"},{1},.5,1,.1,names));
  EXPECT_FALSE(input.accept({"lh"},{1},1.1,1,.1,names));
  EXPECT_FALSE(input.accept({"lh","lh"},{1,2},.99,1,.1,names));
  EXPECT_TRUE(input.accept({"rk","lh","ra"},{2,1,std::numeric_limits<double>::quiet_NaN()},.99,1,.1,names));
  EXPECT_DOUBLE_EQ(input.frame().position[4],2);
  EXPECT_TRUE(input.frame().valid[0]);
  EXPECT_FALSE(input.frame().valid[1]);
  EXPECT_FALSE(input.frame().valid[5]);
  EXPECT_FALSE(input.accept({"lh"},{3},.99,1,.1,names));
  EXPECT_TRUE(input.fresh(1,.1));
  EXPECT_FALSE(input.fresh(1.2,.1));
  EXPECT_DOUBLE_EQ(input.frame().position[0],1);
  LearningRates rates;
  EXPECT_DOUBLE_EQ(rates.phase_rate,0);
  EXPECT_DOUBLE_EQ(rates.coefficient_rates[5].x[3],0);
}

TEST(Transition, PositiveScaleRetargetHasConsistentDerivative) {
  PositiveTransition curve;
  curve.set(.1,0,.5);
  const auto before=curve.sample(.2);
  curve.set(2,.2,.5);
  EXPECT_NEAR(before.rate,curve.sample(.2).rate,1e-12);
  for (double t=.21;t<.7;t+=.01) {
    EXPECT_GT(curve.sample(t).value,0);
    EXPECT_NEAR(curve.sample(t).rate,
      (curve.sample(t+1e-6).value-curve.sample(t-1e-6).value)/2e-6,1e-7);
  }
}
