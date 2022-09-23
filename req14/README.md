# Notes

## Alex
Safety: self-driving car demos feature a 'big read button' which enables manual control of the car in case of technology failure.
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