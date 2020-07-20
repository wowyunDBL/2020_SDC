# 2020_SDC

### set parameter in this way
```cpp
int main(int argc, char **argv)
{
	ros::init(argc, argv, "easy_node");
	bool flag = false;
	for (int i = 1; i < argc; i++) 
	{
	    std::string filename = std::string(argv[i]);
	    if (filename == "ITRI_Private_1") {
	        Localize lObj("ITRI_Private_1", 3.14);
	        flag = true;
	    }
	    else if (filename == "ITRI_Private_2") {
	        Localize lObj("ITRI_Private_2", 2.355);
	        flag = true;
        }
	    else if (filename == "ITRI_Private_3") {
	        Localize lObj("ITRI_Private_3", 1.57);
	        flag = true;
        }
	    else if (filename == "ITRI_Public") {
	        Localize lObj("ITRI_Public", 0);
	        flag = true;
	    }
	}
	if (flag == false) {
	    std::cout << "usage: rosrun easy easy_node BAGNAME [BAGNAME ...]\n";
	    std::cout << "*** Please DON'T ADD .bag behind the bag name! ***\n";
	    std::cout << "*** BAGNAME: ITRI_Public/ITRI_Private_1/ITRI_Private_1/ITRI_Private_3 ***\n";
	}

	return 0;
}

```
