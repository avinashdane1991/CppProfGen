# cyclic-app-template-cpp

This project was bootstrapped using [the official Mosaiq Cyclic App template for C++](https://github.com/mosaiqone/cyclic-app-template-cpp).

## Developer instructions 👩‍💻👨‍💻

* Source code files are placed inside `src`
* Run build via VS Code _Terminal_ ➡ _Run Task_ ➡ _Build_
* The build result is located at `build/lib/libcyclicapp.so`

### Advanced/Optional 🕵️‍♀️🕵️‍♂️

* Update binary name:
	* ⚠️ The binary name does not matter much, the name in App Inventory has precedence
	* In `CMakeLists.txt` replace `cyclicapp` (line 4) by a custom name
	* In `.devcontainer.json` replace `"libcyclicapp.so"` (line 9) by `"lib<binary-name>.so"`
