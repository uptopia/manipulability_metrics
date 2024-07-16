/*
 * Filename: manipulability_measure.cpp
 *
 * Copyright 2020 Tecnalia
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <manipulability_metrics/metrics/m_score_measure.h>

#include <Eigen/LU>
#include <iostream>
#include <math.h>

using namespace std;

namespace manipulability_metrics
{
int sign(float val)
{
    int result;
    if (val>0)
        result = 1;
    else if (val <0)
        result = -1;
    else
        result = 0;
    return result;        
}

double cal_joint_lmt_val(const float theta, const float min_deg, const float max_deg)
{
    //設定此單軸旋轉的正負極限
    float tot  = max_deg-min_deg;
    float half = max_deg-(tot/2);
    float tmp_w = (sign(theta-min_deg)*sign(max_deg-theta))-1.0;
    //單位步階函數(unit step function)
    float w;
    if(tmp_w >= 0)
        w = 1.0;
    else
        w = 0.0;
    //w = hardlim((sign(n-min_deg)*sign(max_deg-n))-1); #單位步階函數(unit step function)

    //【Method 3】JL eval method another view (with abs)
    float tmp = (theta-half) / (max_deg-half);
    float score = (1.0-pow(tmp,4))*w;

    return score;
}

double mScoreMeasure(const KDL::JntArray& joint_positions)
{
    std::vector<float> JL_score = {};
    std::vector<float> Sshoulder_score = {};
    std::vector<float> Selbow_score = {};
    std::vector<float> Swrist_score = {};

    // joint_positions in radian
    float tot_joint_num = joint_positions.rows();//.columns();
    for(int cnt=0; cnt<tot_joint_num; cnt++)
    {
        float ang = joint_positions(cnt)*360.0/M_PI;
        float min_angle, max_angle;
        cout <<"joint #:"<< cnt <<"; joint_angle: "<< ang <<endl;

        //Compute Joint-Limits Score (JL-score)
        JL_score.push_back(cal_joint_lmt_val(ang, -360, 360));

        //Compute Singularity Score (S-score) Singularity Shoulder
        float val_shoulder;
        if(cnt ==2 || cnt ==3 || cnt ==4){
            if (ang<0 && ang >= -360){
                min_angle = -360;
                max_angle = 0;
            }   
            else{
                min_angle = 0;
                max_angle = 360;
            }
            val_shoulder =cal_joint_lmt_val(ang, min_angle, max_angle);
        }
        else
            val_shoulder =100;
        Sshoulder_score.push_back(val_shoulder);

        //Compute Singularity Score (S-score) Singularity Elbow
        float val_elbow;
        if(cnt ==3){
            if (ang<360 && ang >= 180){
                min_angle = 180;
                max_angle = 360;
            }   
            else if (ang<180 && ang >= 0){
                min_angle = 0;
                max_angle = 180;
            }   
            else if (ang<0 && ang >= -180){
                min_angle = -180;
                max_angle = 0;
            }   
            else{
                min_angle = -180;
                max_angle = -360;
            }   
            val_elbow =cal_joint_lmt_val(ang, min_angle, max_angle);
        }
        else
            val_elbow =100;
        Selbow_score.push_back(val_elbow);

        //Compute Singularity Score (S-score) Singularity Wrist
        float val_wrist;
        if (cnt ==5){
            if (ang<360 && ang >= 180){
                min_angle = 180;
                max_angle = 360;
            }        
            else if (ang<180 && ang >= 0){
                min_angle = 0;
                max_angle = 180;
            }   
            else if (ang<0 && ang >= -180){
                min_angle = -180;
                max_angle = -0;
            }   
            else{
                min_angle = -180;
                max_angle = -360;
            }   
            val_wrist =cal_joint_lmt_val(ang, min_angle, max_angle);
        }
        else
            val_wrist =100;
        Swrist_score.push_back(val_wrist);
    }

    // Compute Manipulability Score (M-score)
    float min_JL_score = *min_element(JL_score.begin(), JL_score.end());
    float min_Sshoulder_score = *min_element(Sshoulder_score.begin(), Sshoulder_score.end());
    float min_Selbow_score = *min_element(Selbow_score.begin(), Selbow_score.end());
    float min_Swrist_score = *min_element(Swrist_score.begin(), Swrist_score.end());
    float M_score = std::min({min_JL_score, min_Sshoulder_score, min_Selbow_score, min_Swrist_score});

    cout << "JL_score:"        << min_JL_score        << "\n\t";
    for_each(JL_score.begin(), JL_score.end(), [](int x) { cout << x << " "; });    
    cout << "\nSshoulder_score:" << min_Sshoulder_score << "\n\t";
    for_each(Sshoulder_score.begin(), Sshoulder_score.end(), [](int x) { cout << x << " "; });    
    cout << "\nSelbow_score:"    << min_Selbow_score    << "\n\t";
    for_each(Selbow_score.begin(), Selbow_score.end(), [](int x) { cout << x << " "; });    
    cout << "\nSwrist_score:"    << min_Swrist_score    << "\n\t";
    for_each(Swrist_score.begin(), Swrist_score.end(), [](int x) { cout << x << " "; });    
    cout << "\nM_score:"         << M_score <<endl;

    return M_score;
}
}  // namespace manipulability_metrics
