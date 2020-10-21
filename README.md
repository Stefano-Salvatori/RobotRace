# RobotRace

## Compile and execute
1. Move into the project folder and create a build directory
```
mkdir build
cd build
```

2. Compile and make the c/c++ binaries
```
cmake ..
make
cd ..
```

3. Run the Single Bot genetic algorithm with (this will create a best_single folder containing the parameters of the best genome foreach generation)
```
./build/SingleRobotExperiment
```
**n.b**: You can stop the genetic algorithm whenever you want but remember to change the test.argos file setting the correct parameters to use: you can do it by changing 
```xml
<params num_inputs="24" num_outputs="2" parameter_file="best_single/best_X.dat" />
```
where X is the generation you stopped at

4. Test the results using Argos
```
argos3 -c test.argos
```

