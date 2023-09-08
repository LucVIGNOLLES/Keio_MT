Hey ! 

I put the main files that will be useful to you in this folder. 

The `cam_generation` folder contains the code that implements the cam generation method.

	- `cam_gen.py` is the main generation file. Refer to the comments in the code to understand the workings of it
	- `cam_gen_animated.py` does the same things, but it also creates and animation of how things work
	- `analysis.py` allows to check if a generated cam has the right geometric behavior

The `data_analysis` folder contains code used to make sense of the raw data obtained during experiment.
The filenames are self explenatory, but there are some special files:

	- `experimental_data.py` contains a class that makes itt easy to extract data from the .csv files that contains the raw data
	- `tsa.py` contains useful functions about the TSA
	- Other files are for generating graphs of the data

You can either build upon these files, or remake the code in a language you are more comfortable with, like matlab. It is up to you ! 