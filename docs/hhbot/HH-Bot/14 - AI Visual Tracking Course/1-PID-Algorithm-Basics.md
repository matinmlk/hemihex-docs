---
title: 1.PID algorithm basics
sidebar_position: 1
---

### 1.1  Introduction

PID is to perform proportional integral differential operation on the input deviation, and the superposition result of the operation is used to control the actuator. The formula is as follows:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140001.png)

It consists of three parts:

- P is the proportion, which is the input deviation multiplied by a coefficient;
- I is the integral, which is the integral operation of the input deviation;
- D is differentiation, which performs differential operations on the input deviation.

As shown below is a basic PID controller:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140002.png)

(1) Proportional part

The mathematical expression of the proportional part is: Kp*e(t)

In an analog PID controller, the role of the proportional link is to react instantaneously to deviations. Once the deviation occurs, the controller immediately takes control to change the control quantity in the direction of reducing the deviation. The strength of the control effect depends on the proportional coefficient,The larger the proportional coefficient, the stronger the control effect, the faster the transition process, and the smaller the static deviation of the control process;But the larger it is, the easier it is to produce oscillations and destroy the stability of the system. Therefore, the sample coefficient must be selected appropriately to achieve a short transition time, small static difference and stable effect.

Advantages: Adjust the open-loop proportional coefficient of the system, improve the steady-state accuracy of the system, reduce the inertia of the system, and speed up the response speed.

Disadvantages: Using only P controller, an excessively large open-loop proportional coefficient will not only increase the overshoot of the system, but also make the system stability margin smaller and even unstable.

(2) Points part

The mathematical expression of the integral part is:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140003.png)

The larger the integral constant, the weaker the accumulation effect of the integral, and the system will not oscillate during the transition;However, increasing the integral constant will slow down the static error elimination process, and the time required to eliminate the deviation is also longer, but it can reduce overshoot and improve the stability of the system. When Ti is small, the integral effect is strong. At this time, oscillation may occur in the system transition time, but the time required to eliminate the deviation is shorter. Therefore, Ti must be determined according to the specific requirements of actual control.

Advantages: Eliminate steady-state errors.

Disadvantages: The addition of the integral controller will affect the stability of the system and reduce the stability margin of the system.

(3) Differential part

The mathematical expression of the differential part is:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140004.png)

The function of the differential link is to prevent the change of deviation. It is controlled based on the change trend (change speed) of the deviation. The faster the deviation changes, the larger the output of the differential controller is and the correction can be made before the deviation becomes larger. The introduction of differential action will help reduce overshoot, overcome oscillation, and stabilize the system. It is especially beneficial to high-order systems because it speeds up the tracking speed of the system. However, the effect of differential is very sensitive to the noise of the input signal. For systems with larger noise, differential is generally not used, or the input signal is filtered before differential takes effect. The role of the differential part is determined by the differential time constant Td. The larger the Td, the stronger its effect in suppressing deviation changes; the smaller the Td, the weaker its effect in resisting deviation changes. The differential part obviously plays a big role in the stability of the system. Appropriate selection of the differential constant Td can optimize the differential effect.

Advantages: It makes the response speed of the system faster, reduces overshoot, alleviates oscillation, and has a "predictive" effect on the dynamic process.

### 1.2 Selection of PID algorithm

Digital PID control algorithms can be divided into positional PID and incremental PID control algorithms. So before we decide which PID algorithm to use, we should first understand its principles:

- Positional PID algorithm:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140005.png)

e(k): User-set value (target value) - the current status value of the control object

Proportion P : e(k)

Integral I : ∑e(i) Accumulation of errors

Differential D : e(k) - e(k-1) this time error - last time error

That is, positional PID is the actual position of the current system, and the deviation from the expected position you want to achieve is used for PID control.

Because there is an error integral ∑e(i), which is always accumulated, that is, the current output u(k) is related to all past states, and the accumulated value of the error is used;(The error e will be accumulated), the output u(k) corresponds to the actual position of the actuator,Once the control output goes wrong (there is a problem with the current state value of the control object), a large change in u(k) will cause a large change in the system. When the positional PID reaches saturation in the integral term, the error will still continue to accumulate under the action of the integral.Once the error starts to change in the opposite direction, the system needs a certain amount of time to exit the saturation zone, so when u(k) reaches the maximum and minimum, the integral function must be stopped,And there must be integral limiting and output limiting, so when using positional PID, generally we use PD control directly and positional PID is suitable for objects whose actuators do not have integral components.Such as steering gear and balance car upright and temperature control system control

Advantages: Positional PID is a non-recursive algorithm that can directly control the actuator (such as a balancing car).The value of u(k) has a one-to-one correspondence with the actual position of the actuator (such as the current angle of the car), so it can be used well in objects where the actuator does not have an integral component.

Disadvantages: Each output is related to the past state. When calculating, e(k) needs to be accumulated, which requires a large amount of calculation work.

- Incremental PID:

![Diagram](/img/docs/hhbot/14-ai-visual-tracking-course/14-1/202309140006.png)

Proportion P: e(k)-e(k-1) error this time - error last time

Integral I: e(k) error

Differential D: e(k)-2e(k-1)+e(k-2)this time error-2*last error+last time error

Incremental PID can be well seen from the formula. Once KP, TI and TD are determined, as long as the deviation of the three measured values before and after is used,That is, the control amount $Delta u(k)$ obtained by calculating the control increment from the formula corresponds to the increment of the position error in recent times,Instead of corresponding to the deviation from the actual position, there is no error accumulation. That is to say, there is no need for accumulation in incremental PID.The determination of the control increment $Delta u(k)$ is only related to the latest three sampling values. It is easy to obtain better control effects through weighted processing, and when a problem occurs in the system, the incremental method will not seriously affect the work of the system.

Summary: Incremental PID is an increment to positional PID.At this time, the controller outputs the difference between the position values calculated at two adjacent sampling moments. The result is an increment, that is, the control amount needs to be increased (negative value means decrease) based on the last control amount.

Advantages: ① The impact of malfunction is small. If necessary, the erroneous data can be removed by logical judgment.

②The impact is small when switching between manual and automatic, making it easy to achieve disturbance-free switching. When the computer fails, the original value can still be maintained.

③ No accumulation is required in the calculation formula. The determination of the control increment Δu(k) is only related to the latest three sampling values.

Disadvantages: ① The integral truncation effect is large and there is a steady-state error;

②The impact of overflow is large. It is not good to use incremental method for some controlled objects;

- Difference between incremental and positional

(1) The incremental algorithm does not require accumulation. The determination of the increment of the control volume is only related to the recent deviation sampling values. The calculation error has a small impact on the calculation of the control volume. The positional algorithm uses the accumulated value of past deviations, which is prone to large accumulated errors.

(2) The incremental algorithm obtains the increment of the control amount. For example, in valve control, only the change in valve opening is output, and the impact of malfunction is small. If necessary, logical judgment can be used to limit or prohibit this time. The output will not seriously affect the work of the system. The positional output directly corresponds to the output of the object, so it has a greater impact on the system.

(3) The incremental PID control output is an increment of the control amount and has no integral effect. Therefore, this method is suitable for objects with integral components in the actuator, such as stepper motors, etc., while the positional PID is suitable for objects with no integral components in the actuator. Objects with integral components, such as electro-hydraulic servo valves.

(4) When performing PID control, positional PID requires integral limiting and output limiting, while incremental PID only needs to output limiting. Positional PID and incremental PID are just two implementations of digital PID control algorithms. Just the form, the essence is exactly the same. The main difference is that the integral items are stored in different ways. The positional PID integral items are stored separately, while the incremental PID integral items are stored as part of the output. Various players on the Internet also have their own unique insights and opinions on the use of positional and incremental PIDs. The specific opinion depends on which algorithm is suitable for our specific application scenarios.

### 1.3 Debugging PID parameters

There are many methods for selecting parameters of PID controller, such as trial and error method, critical proportional method, extended critical proportional method, etc. However, for PID control, parameter selection is always a very complicated task, and continuous adjustments are required to obtain a more satisfactory control effect. Based on experience, the general steps for determining PID parameters are as follows:

- Determine proportional coefficient Kp

When determining the proportional coefficient Kp, first remove the integral term and differential term of the PID, you can set Ti=0, Td=0, making it

Pure proportional adjustment. The input is set to 60% to 70% of the maximum allowable output of the system. The proportional coefficient Kp gradually increases from 0 until the system oscillates; conversely, the proportional coefficient Kp gradually decreases from this time until the system oscillation disappears. Record the proportional coefficient Kp at this time, and set the PID proportional coefficient Kp to 60% to 70% of the current value.

- Determine the integration time constant Ti After the proportional coefficient Kp is determined, set a larger integration time constant Ti, then gradually reduce Ti until the system oscillates, and then in turn, gradually increase Ti until the system oscillation disappears. Record the Ti at this time, and set the PID integration time constant Ti to 150% to 180% of the current value.
- Determine the differential time constant Td The differential time constant Td generally does not need to be set, it can be 0. At this time, PID adjustment is converted into PI adjustment. If it needs to be set, it is the same as the method for determining Kp, taking 30% of its value when there is no oscillation.

Of course, this is just my personal debugging method, and may not be suitable for everyone and every environment. It is only provided for your reference; however, there are also classic trial tips for debugging PID circulating on the Internet, and I also posted them for your reference:

Find the best parameter setting, and check in order from small to large.

First the proportion, then the integral, and finally the differential.

The curve oscillates frequently, and the proportional dial needs to be enlarged.

The curve floats around the big bend, and the proportion dial is turned smaller.

The curve deviates and recovers slowly, and the integration time decreases.

The curve fluctuation period is long, and the integration time will be longer.

The oscillation frequency of the curve is fast, so first reduce the differential.

If the dynamic difference is large, the fluctuation will be slow, and the differential time should be lengthened.

The ideal curve has two waves, the front is high and the back is low four to one.

One look, two adjustments and multiple analyses, the adjustment quality will not be low.

PID is a proportional (P), integral (I), and differential (D) control algorithm. It is not necessary to have these three algorithms at the same time. It can also be PD, PI, or even only P algorithm control. One of the simplest ideas I had for closed-loop control was P control, which feeds back the current result and then subtracts it from the target. If it is positive, it will slow down, and if it is negative, it will speed up. Of course, this is just the simplest closed-loop control algorithm. , that is, let’s go back to the summary of positional and incremental in the previous section. For details, we must refer to our current control environment. Because of the differences in each control system, the parameters that can allow our system to achieve the most stable effect are certainly OK. .
