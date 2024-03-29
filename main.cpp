#include "CommonProcesses.h"
#include "Region_Growing.h"
#include "RAN_SA_C.h"
#include "Segmentation.h"

int main() {

	int reg_or_ran;
	int sor;
	string file;

	cout << "Choose your segmentation method:" << endl
		 << "***********************" << endl
		 << "1. Region Growing" << endl
		 << "2. Random Sample Consensus" << endl
		 << "***********************" << endl;
	cin >> reg_or_ran;
	while (1) {
		if (reg_or_ran == 1 || reg_or_ran == 2) break;
		else 
		{
			cout<<("You've selected invalid choice Please enter a value in the range of [1,2]")<<endl;
			cin >> reg_or_ran;
		}
	}
	
		
	cout << "Choose your raw cloud file:" << endl
		<< "********************************" << endl
		<< "1. depth_image0101_sampling.pcd" << endl
		<< "2. depth_image0118_sampling.pcd" << endl
		<< "3. depth_image0303_sampling.pcd" << endl
		<< "********************************" << endl;

	cin >> sor;
	while (1) {
		if (sor == 1 || sor == 2 || sor == 3) break;
		else
		{
			cout << ("You've selected invalid choice Please enter a value in the range of [1,3]") << endl;
			cin >> sor;
		}
	}

	switch (sor) {
	case 1:
		file = "depth_image0101_sampling.pcd";
		break;
	case 2:
		file = "depth_image0118_sampling.pcd";
		break;
	case 3:
		file = "depth_image0303_sampling.pcd";
		break;
	}

	if(reg_or_ran ==1)
	{
		cout << "Please enter what you want after region growing segmentation applied on original cloud:" << endl;
		Region_Growing regionn(file);	
	}
	else if (reg_or_ran == 2)
	{	
		cout << "Please enter what you want to do after RANSAC Segmentation applied on the Original Cloud:" << endl;
		RAN_SA_C ransacc(file);		
	}
	
	
	return 0;

}