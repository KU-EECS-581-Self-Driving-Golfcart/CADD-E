# DeepRacer

During Sprint 5, Dr. Johnson notified us that before we would be able to purchase a real golf cart, we would need to demonstrate the capabilities of our software on an AWS DeepRacer car. The initial steps of getting our software running on the DeepRacer have been as follows:

- Check out an AWS DeepRacer from the EECS Shop
- Read up on AWS DeepRacer documentation:
  - [What is AWS DeepRacer?](https://docs.aws.amazon.com/deepracer/latest/developerguide/what-is-deepracer.html)
  - [Get Started with AWS DeepRacer](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-get-started.html)
  - [Drive your AWS DeepRacer Vehicle](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-drive-your-vehicle.html)
  - [An Advanced Guide to AWS DeepRacer](https://towardsdatascience.com/an-advanced-guide-to-aws-deepracer-2b462c37eea)
- Ask EECS Shop about the root password to the compute module (I was told that they didn't know so I had to reformat the compute module)
- Reformat the Compute Module (following [this](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html) guide)
  - Wipe my USB Drive
  - Partition my USB Drive
  - Load a customized Ubuntu ISO file onto the BOOT partition using [UNetbootin](https://unetbootin.github.io)
  - Load an updated software package onto the Data partition
  - Reboot the DeepRacer copmute module from BIOS, booting with the BOOT partition on my USB drive.
- Test controlling the car (manually)
  - Following steps from [Drive your AWS DeepRacer Vehicle](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-drive-your-vehicle.html), I was able to "drive" the AWS DeepRacer with manual controls. This helped show that the car's mechanical parts were still operating properly and were capable of receiving commands from some type of source.
- Test controlling the car (programatically)
  - I tested two different third-party APIs ([API 1](https://github.com/thu2004/deepracer-vehicle-api), [API 2](https://github.com/lshw54/deepracer_api)) on the DeepRacer. Both were able to connect to the car (that is, they received HTTP packets back from the car indicating successful acknowledgement) but neither were able to actually control the car. Next sprint, I will continue my work investigating programmatic control of the DeepRacer car so that we may be able to control it with our autonomous system.