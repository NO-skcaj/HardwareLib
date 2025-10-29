// #pragma once

// #include "lib/hardware/vision/PhotonVision.h"


// class Vision : public hardware::vision::PhotonVision, public frc2::SubsystemBase
// {
//     public:

//         static hardware::vision::PhotonVision* GetInstance()
//         {
//             static Vision instance;
//             static Vision* instancePtr;
//             return instancePtr;
//         }

//     private:

//         Vision()
//         {
//             // Initialize the PhotonVision
//             hardware::vision::PhotonVision::PhotonVision();
//         }
// };