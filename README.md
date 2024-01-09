# Howdy Bots Crescendo 2024 Bot Java

[![CI](https://github.com/frc6377/crescendo_2024/actions/workflows/main.yml/badge.svg)](https://github.com/frc6377/crescendo_2024/actions/workflows/main.yml)

## References

To get started with Command-base, start with [What Is “Command-Based” Programming?](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)

Our repository attempts to follow the best practices listed in the document
[Structuring a Command-Based Robot Project](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html).

For organization guidelines read: [Organizing Command-Based Robot Projects](https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html).


## Appendix A: Gradle Commands

This commandline commands can make it easier to work with our system. For instance, you can apply spotless and build at the same time by doing:

```
./gradlew spotlessApply build
```

### [Gradlew Tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html#gradlew-tasks)
This article aims to highlight the gradle commands supported by the WPILib team for user use. These commands can be viewed by typing ./gradlew tasks at the root of your robot project. Not all commands shown in ./gradlew tasks and unsupported commands will not be documented here.

### [Spotless tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/code-formatting.html#running-spotless)

Spotless can be ran using ./gradlew spotlessApply which will apply all formatting options. You can also specify a specific task by just adding the name of formatter. An example is `./gradlew spotlessmiscApply`.

Spotless can also be used as a CI check. The check is ran with `./gradlew spotlessCheck`.

### [Build tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html#build-tasks)
`./gradlew build` - Assembles and tests this project. Useful for prebuilding your project without deploying to the roboRIO. ./gradlew clean - Deletes the build directory.

### [CompileCommands tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html#compilecommands-tasks)
`./gradlew generateCompileCommands` - Generate compile_commands.json. This is a configuration file that is supported by many Integrated Development Environments.

### [EmbeddedTools tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html#embeddedtools-tasks)
`./gradlew deploy` - Deploy all artifacts on all targets. This will deploy your robot project to the available targets (IE, roboRIO).

`./gradlew discoverRoborio` - Determine the address(es) of target roboRIO. This will print out the IP address of a connected roboRIO.

### [GradleRIO tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html#gradlerio-tasks)
`./gradlew downloadAll` - Download all dependencies that may be used by this project

`./gradlew $TOOL$` - Runs the tool $TOOL$ (Replace $TOOL$ with the name of the tool. IE, Glass, Shuffleboard, etc)

`./gradlew $TOOL$Install` - Installs the tool $TOOL$ (Replace $TOOL$ with the name of the tool. IE, Glass, Shuffleboard, etc)

`./gradlew InstallAllTools` - Installs all available tools. This excludes the development environment such as Visual Studio Code. It’s the users requirement to ensure the required dependencies (Java) is installed. Only recommended for advanced users!

`./gradlew simulateExternalCpp` - Simulate External Task for native executable. Exports a JSON file for use by editors / tools

`./gradlew simulateExternalJava` - Simulate External Task for Java/Kotlin/JVM. Exports a JSON file for use by editors / tools

`./gradlew simulateJava` - Launches simulation for the Java projects

`./gradlew simulateNative` - Launches simulation for C++ projects

`./gradlew vendordep` - Install vendordep JSON file from URL or local installation. See 3rd Party Libraries
