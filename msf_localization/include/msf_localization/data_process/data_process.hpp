#ifndef MSF_LOCALIZATION_DATA_PROCESS_DATA_PROCESS_HPP_
#define MSF_LOCALIZATION_DATA_PROCESS_DATA_PROCESS_HPP_




namespace msf_localization{

class DataProcess{
public:
    DataProcess();
    ~DataProcess();

public:
    bool Predict();
    bool Correct();


private:
    double time;
    std::deque<IMUData> raw_imu_buff_;
    


};



}


#endif