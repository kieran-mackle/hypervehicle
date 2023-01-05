# Getting Started with *hypervehicle*

## Dependencies

### GDTK
*Hypervehicle* depends on the 
[GDTK](https://github.com/gdtk-uq/gdtk) Python 
package for its geometric toolkit. Note that a full install is 
not required. Instead, do a 
[sparse checkout](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository)
of the relevant files, using the commands below.

```
mkdir gdtk
cd gdtk/
git init
git remote add -f origin https://github.com/gdtk-uq/gdtk.git
git config core.sparseCheckout true
echo "src/lib/" >> .git/info/sparse-checkout
git pull origin master
cd src/lib
python3 -m pip install .
cd ../../../
```



## Installation
First clone the repository, then install via pip.

```
git clone https://github.com/kieran-mackle/hypervehicle
cd hypervehicle
python3 -m pip install ./
```

### Testing the installation
```
python3 -m pytest tests/
```


## Example
See an example of using *hypervehicle* [here](examples/x43.md).
