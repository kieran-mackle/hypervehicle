
# HYPERVEHICLE: Parametric Vehicle Geometry Generation
A suite of tools to rapidly generate parametric geometry for hypersonic vehicles. Check out the [hypervehicle hangar](docs/hangar.md)
for some examples.

[![x43](https://user-images.githubusercontent.com/60687606/168926371-a383434b-3ea5-40ab-989a-93f7a8d7b4ff.png)](docs/hangar.md)

## Getting Started

### Dependencies
Hypervehicle relies on the [Eilmer](https://github.com/gdtk-uq/gdtk) geometry 
package. Note that a full Eilmer install is not required. Instead, do a 
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
python3 -m pip install -e ./
cd ../../../
```



### Installation

First clone the repository, then install via pip as an editable install.

```
git clone https://github.com/kieran-mackle/hypervehicle
cd hypervehicle
python3 -m pip install -e ./
```


### Testing your install
```
python3 -m pytest tests/
```



## Documentation
HYPERVEHICLE is well documented in-code. To access more information on a 
specific method, use the `help()` method of Python. To view documentation 
of the entire codebase in html format, run the commands below.

```
cd docs
make html
xdg-open _build/html/index.html
```


## Example Vehicle Generation
Please see the [example geometry generation](docs/example.md) to generate a mockup of the X-43A, shown above.




