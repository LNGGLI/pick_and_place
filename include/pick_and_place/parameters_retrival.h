#pragma once


// Posizioni note [m]

TooN::Vector<3, double> High_center;

TooN::Vector<3, double> BH1_S; 
TooN::Vector<3, double> BH2_S;
TooN::Vector<3, double> BH3_S;
TooN::Vector<3, double> BH4_S; 
TooN::Vector<3, double> BH1_G;
TooN::Vector<3, double> BH2_G;
TooN::Vector<3, double> BH3_G;
TooN::Vector<3, double> BH4_G;    


bool retrive_params(ros::NodeHandle nh){

  std::vector<double> High_center_local;
  std::vector<double> BH1_S_local;
  std::vector<double> BH2_S_local;
  std::vector<double> BH3_S_local;
  std::vector<double> BH4_S_local;
  std::vector<double> BH1_G_local;
  std::vector<double> BH2_G_local;
  std::vector<double> BH3_G_local;
  std::vector<double> BH4_G_local;

  // High_center
  if (!nh.getParam("High_center", High_center_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      High_center[i] = High_center_local[i];

  // BH1_S
  if (!nh.getParam("BH1_S", BH1_S_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH1_S[i] = BH1_S_local[i];

  // BH1_G
  if (!nh.getParam("BH1_G", BH1_G_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH1_G[i] = BH1_G_local[i];

  // BH2_S
  if (!nh.getParam("BH2_S", BH2_S_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH2_S[i] = BH2_S_local[i];

  // BH2_G
  if (!nh.getParam("BH2_G", BH2_G_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH2_G[i] = BH2_G_local[i];

  // BH3_S
  if (!nh.getParam("BH3_S", BH3_S_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH3_S[i] = BH3_S_local[i];

  // BH3_G
  if (!nh.getParam("BH3_G", BH3_G_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH3_G[i] = BH3_G_local[i];

  // BH4_S
  if (!nh.getParam("BH4_S", BH4_S_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH4_S[i] = BH4_S_local[i];

  // BH4_G
  if (!nh.getParam("BH4_G", BH4_G_local)) {
    std::cout << "Ritiro dei parametri fallito";
    return false;
  }
  for(int i = 0; i < 3; i++)
      BH4_G[i] = BH4_G_local[i];

  
  
  
}