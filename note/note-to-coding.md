# coding-note to C++

### set parameter in C++
```cpp
int main(int argc, char **argv)
{
	ros::init(argc, argv, "argument_node");
	bool flag = false;
	for (int i = 1; i < argc; i++) 
	{
	    std::string filename = std::string(argv[i]);
	    if (filename == "rosbag_1") {
	        Localize lObj("rosbag_1", 3.14);
	        flag = true;
	    }
	}
	if (flag == false) {
	    std::cout << "usage: rosrun easy easy_node BAGNAME [BAGNAME ...]\n";
	    std::cout << "*** Please DON'T ADD .bag behind the bag name! ***\n";
	    std::cout << "*** BAGNAME: rosbag_1 ***\n";
	}

	return 0;
}
```

### for python
* run python file in this way
```bash
$ python read_bag.py --bag [bagName].bag
``` 
* code implementation
```python
from argparse import ArgumentParser
parser = ArgumentParser(description='Input bag file name')
parser.add_argument('--bag', metavar='FILE', type=str, help='input bag file name', nargs='+')
file_bags = parser.parse_args().bag
```
