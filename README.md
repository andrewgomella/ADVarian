# README # (Work in progress)
July 2015, Andrew Gomella

This is an areadetector driver which is meant to work with the current git version of areadetector. https://github.com/areaDetector/areaDetector

The folder ADVarian is meant to drop into the areadetector folder like the other camera drivers. Up until now it was mostly written and tested with (list version). Windows 7 x64, Visual Studio 12. 


### How do I get set up? ###

-Compilation of this driver depends on areadetector being compiled along with the various libraries required by area detector. 
-It depends on the header files from Varian(which are included), the dll files from Varian as well VirtCP64.lib 

-Compiling is done by running vcvarsall_x64.bat and then make clean && make. 

- The varianconfig command in the st.cmd file needs to put to the folder setup with the varian config files appropriate for the detector. 

### Who do I talk to? ###

andrewgomella@gmail.com