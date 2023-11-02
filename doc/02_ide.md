# 2. IDE & Debugging
- [2. IDE & Debugging](#2-ide--debugging)
  - [2.1. Pycharm](#21-pycharm)
    - [2.1.1. Terminal Workaround](#211-terminal-workaround)
    - [2.1.2. Intended Setup by Jetbrains](#212-intended-setup-by-jetbrains)
  - [2.2. VS Code](#22-vs-code)
  - [2.3. Debugging](#23-debugging)

As with all software development projects, we recommend you use an integrated development environment (IDE). For python projects we recommend Pycharm or VS Code.

## 2.1. Pycharm
Pycharm is a commercial python IDE developed by Jetbrains. You can use your `@student.kit.edu` email address to register at their site and get a free educational license for their "Professional Edition". The Community Edition is free, but has fewer features (though you probably won't need them). Pycharm has more "insightful" python features, but is slower than VS Code.

Pycharm ignores the environment variables you have set during the installation process. You therefore need to use one of the following two workarounds to be able to execute / debug your programs in Pycharm.

### 2.1.1. Terminal Workaround
If you start Pycharm from a terminal with the correct environment variables loaded, instead of using a Desktop Shortcut, it will work. E.g., in your terminal

```bash
# activate your conda environment with the env vars
conda activate SimulationFramework

# start pycharm 
pycharm-professional # or pycharm-community
```

### 2.1.2. Intended Setup by Jetbrains
As this requires some navigation through the GUI, we recommend you take a look at [this StackOverflow thread](https://stackoverflow.com/questions/42708389/how-to-set-environment-variables-in-pycharm) with Screenshots.

In the same menu you can set a Default Configuration / Template(?), so that you do not edit the run configuration for each script you run separately.

## 2.2. VS Code
VS Code is an open source IDE developed by Microsoft. It is highly configurable with many available extensions. Generally it is faster than Pycharm. The latest updates have brought VS Code nearly on par with Pycharm with regard to features, but Pycharm is still a bit ahead in some areas.

As VS Code, to the best of our knowledge, runs everything in a common terminal, you should not need any special accommodations to execute your scripts.

## 2.3. Debugging
Debugging scripts using mujoco-py might behave strange, especially in Pycharm.

The debugger might stop before reaching your first breakpoint, showing the exception `name 'raw_input' is not defined` or similar.

Simply RESUME the debug execution. You might need to do this 2-3 times at the beginning of each process.

We suspect that Pycharm somehow has the need to debug the external libraries linked in the `LD_PRELOAD` environment variable.


Afterwards everything should behave as expected.

[Back to Overview](./)