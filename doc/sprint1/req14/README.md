# Notes

## Alex
Safety: self-driving car demos feature a 'big red button' which enables manual control of the car in case of technology failure.
- Safety driver is attentive at all times during the demo
- Collect failure modes and track between software versions
- Google: 2bil miles in a simulator before uploading to physical car

For localization, we need 5-10cm level accuracy
- GPS with RTK can achieve this

For simulation, instead of rendering the complete environment, reduce the problem to a 2d environment plus time; then perform many more sims than otherwise possible

Tesla's goal was to drive without maps, but they realized it was too ambitious for a full self-driving car

Awareness of obstacles on and just outside the path
- Each recognized object needs a prediction of its future path
- The more known about an object, the better one can predict its path (human -> human facing left -> human facing left looking away)

100ms is the typical self-driving car cycle rate

## Caden
Object detection:
- Beyond detecting objects in the path (people, vehicles, etc.) it is important to understand a predicted path that that object is taking. If a person is walking into the cart path, we would want to take note of this and stop until they are no longer in the path, however if a person is stationary on the side of the path it would be okay to proceed by decreasing speed (but not entirely stopping).
- When attempting to predict an object's path, it is important to recognize what kind of object it is and what sort of paths it may take. People may move in any direction but are most likely going to move in the direction they are facing. Cars on the other hand will move either forward or backward relative to the direction they are facing (though they may turn according to the rotation of the front tires). Cars will NOT move sideways.

Physical implementation:
- The mechanical problem is critical to the success of an autonomous vehicle. If the vehicle cannot precisely move in the direction it wants to move in, the software behind the autonomy will be useless.
- Control of the brake and accelerator are easier than control of steering.
- It is important to have a driver ready to take over at all times until the vehicle has been very thoroughly tested
  - This may be implemented with a "big red button" or some way for the driver to cease autonomous control and control the car manually in a moment's notice.

Open Source Implementations:
- [Apollo](https://github.com/ApolloAuto/apollo) is an open source autonomous vehicle stack. While this may not be 100% applicable to our problem, it will certainly be a good resource to have for reference while developing our software stack.