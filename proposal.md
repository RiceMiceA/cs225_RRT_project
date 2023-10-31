## Project Summary

Our proposed final project is to implement the 'black box' method for searching a large text file for specific patterns.

## **Getting Started**
### **Installing Anaconda**
This repository uses an anaconda environment to manage versions of all dependencies. To get started with installing `conda` please follow [these instructions](https://conda.io/projects/conda/en/latest/user-guide/getting-started.html).
### **Cloning this Repository**
Using the RMDLO Optical Flow repository requires desktop [configuration of a GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

Clone this repository in your desired location by running the following command in a terminal:
```bash
# Clone the repository
$ git clone git@github.com:RMDLO/optical_flow.git
```

## Code

All code files can be found in the `code/` directory. To run the code:

1. Create a `build` directory inside `code/`

2. CD into `build` and run `cmake ..` and `make`.

3. Modify the `main.cpp` file to run whichever `.csv` parsed file you want to time.

## Data

The test data contains RNA sequencing studies of mice with sizes ranging from 100 subsampled kmers to 10000 kmers.

## Documents

The proposal can be found in the `documents/` directory.

## Feedback

All feedback from our project mentor can be found in the `feedback/` directory.
