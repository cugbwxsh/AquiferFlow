# AquiferFlow
AquiferFlow in ModHydro-2007

AquiferFlow is a C/C++ based program in ModHydro-2007, to simulate 
the 3-D variably saturated groundwater flow in the aquifer-systems.
Both ModHydro-2007 and AquiferFlow were developed by Wang Xu-Sheng, in
the China University of Geosciences, Beijing.


/////////////////////////////////////////////////////////////////////////////
History：

ModHydro-2007 was originally developed by Wang (2007) and coded in C/C++ language  
using Microsoft Visual C++ 6.0, including general MFC calsses for hydraulic 
analysis and numerical modeling. 

AquiferFlow was developed by Wang (2007) based on ModHydro-2007 as a program project
in Microsoft Visual C++ 6.0 to perform numerical modeling of groundwater flow.

The codes of AquiferFlow and ModHydro-2007 was firstly published in 2008 on  
http://www.swre.cugb.edu.cn/kxyj/zl/406697.shtml in Chinese.

The program is applied in a case study as reported in Wang et al. (2010).

/////////////////////////////////////////////////////////////////////////////
Run Platform:

1  Windows;
2  Microsoft Visual C++ 6.0;
3  Code in English, explain in Chinese; 

/////////////////////////////////////////////////////////////////////////////
File Structure:

./src        AquiferFlow source files
             ModHydro.h is the header of ModHydro-2007
             ModHydro.cpp  is the classes codes of ModHydro-2007
             AqFlow.dsw is the project file of AquiferFlow on Microsoft Visual C++ 6.0

./bin        Executabe file of the AquiferFlow

/////////////////////////////////////////////////////////////////////////////
References:

http://www.swre.cugb.edu.cn/kxyj/zl/406697.shtml.

Wang, X. S.: AquiferFlow, A finite difference variable saturation three-dimensional aquifer groundwater flow model, China University
of Geosciences (Beijing), Beijing, China, 2007.(in Chinese)

Wang X.-S., M.-G. Ma, X. Li, J. Zhao, P. Dong, and J. Zhou. Groundwater response to leakage of surface water through a thick vadose zone
 in the middle reaches area of Heihe River Basin, in China. Hydrol. Earth Syst. Sci., 2010, 14: 639–650.
