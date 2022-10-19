I initially wanted to create a custom map in CARLA for testing the golf cart design, but several obstacles arose. First, the recommended software for designing maps is MATLAB Roadrunner, for which KU does not have an institutional liscense. Second, the process for importing maps requires immense resources: 600-700GB and 4 hours to perform (carla.readthedocs.io). Since I didn't have the resources to make my own map, I decided it would be valid to test the motion planner in one of the pre-defined CARLA towns, specifically Town07, because it features a long, winding road similar to what we will follow on a golf course. Further, the main purpose of CARLA is too simulate reactions to obstacles like moving pedestrians and barriers, which can be adaquately accomplished using the CARLA api to spawn in and direct the movements of pedestrians. Thus, I will use the town07 map to test our motion planner.