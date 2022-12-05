# Requirement 3
## Get DeepRacer to run using an API

AWS DeepRacer is meant to be used to train reinforcement learning models and does not have a straightforward method for programmatically controlling the car. Because of this, we will need to use a third-party API to control the car.

I tested three different third-party APIs ([API 1](https://github.com/thu2004/deepracer-vehicle-api), [API 2](https://github.com/lshw54/deepracer_api), [API 3](https://github.com/ARCC-RACE/deepracer-rc)) on the DeepRacer. All APIs were able to connect to the car (that is, they received HTTP packets back from the car indicating successful acknowledgement) but neither were able to actually control the car. Luckily, I noticed that [API 2](https://github.com/lshw54/deepracer_api) had a [fork](https://github.com/jacobcantwell/deepracer_api). I tested the forked API because it includes the most recent updates of any of the APIs I had tested thus far. The code worked and was able to send commands to DeepRacer which controlled the car's steering and throttle. I then made a [fork](https://github.com/cskroonenberg/deepracer_api) of my own which is now submoduled in our CADD-E repository.

<video controls>
  <source src="deepracer_test.mov" type="video/mp4">
</video>