#include "CompetitionTask.h"
#include "ProjectService.h"
#include "Servo4.h"
#include "CompetitionQr.h"
#include "VisionAlignment.h"
#include "MaxiCam.h"
#include "usart.h"

void CompetitionTask_Init(void)
{
    CompetitionQr_Init(&huart1);
    VisionAlignment_Init(&huart4);
    ProjectService_Init();
}

void CompetitionTask_Process(void)
{
    CompetitionQr_Process();
    MaxiCam_Process();
    VisionAlignment_Process();
    ProjectService_Run();
}
