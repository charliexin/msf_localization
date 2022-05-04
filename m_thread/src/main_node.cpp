#include <iostream>
#include <thread>
#include<chrono>
 
using namespace std;

int have_new_pose = false;

void process_lidar_data()  //普通的函数，用来执行线程
{
    for (int i = 0; i < 100; ++i)
    {
        cout << "lidar\n";
        
    }
    cout << "process lidar done\n";
    have_new_pose = true;

}

void process_imu_data()  //普通的函数，用来执行线程
{
    for (int i = 0; i < 200; ++i){
        if (have_new_pose) {
            cout << "process_lidar_data before imu data\n";
        }else{
            cout << "process_imu_data\n";
        }
    }


}

int main()
{
    thread thred_lidar(process_lidar_data);
    thread thred_imu(process_imu_data);  

    //thred_lidar.join(); 
    thred_lidar.detach();
    thred_imu.join(); 

    //thred_imu.detach();    
    
 
    return 0;
}