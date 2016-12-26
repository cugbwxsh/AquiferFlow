//####################################################################
//          ModHydro.h
//          Ë®ÎÄÄ£ÐÍÀà¿â³ÌÐò¼¯[MODHYDRO] Í·ÎÄ¼þ
//          °æ±¾:  ¹²Ïí-2007(ÈýÎ¬µØÏÂË®ÓÐÏÞ²î·ÖÄ£ÐÍ)²âÊÔ°æ
//          ¸ÅÊö:  »ùÓÚC/C++/MFCµÄÀà¿â³ÌÐò¼¯, 
//                 ÓÃÓÚÊµÏÖº¬Ë®²ã±ä±¥ºÍ¶ÈµØÏÂË®Á÷µÄÈýÎ¬ÊýÖµÄ£Äâ
//          Éè¼Æ:  ÍõÐñÉý
//          »ú¹¹:  ÖÐ¹úµØÖÊ´óÑ§(±±¾©)Ë®×ÊÔ´Óë»·¾³Ñ§Ôº
//          ----------------------------------------------------------
//          ÁªºÏ×ÊÖú:
//                 Ë®×ÊÔ´Óë»·¾³¹¤³Ì±±¾©ÊÐÖØµãÊµÑéÊÒ
//                 ÖÐ¹ú¿ÆÑ§Ôºº®ÇøºµÇø»·¾³¹¤³ÌÑÐ¾¿Ëù(´´ÐÂ¹¤³ÌÏîÄ¿CACX2003102)
//                 ÖÐ¹ú¹ú¼Ò×ÔÈ»¿ÆÑ§»ù½ðÎ¯(ÏîÄ¿40542011)
//                 Îäºº´óÑ§  Ë®×ÊÔ´ÓëË®µç¹¤³Ì¿ÆÑ§¹ú¼ÒÖØµãÊµÑéÊÒ(¿ª·Å¿ÎÌâ2006B024)
//          ----------------------------------------------------------
//          Éè¼Æ:  2007Äê-2008Äê
//          ·¢²¼:  2008Äê12ÔÂ
//####################################################################

#include "afxtempl.h"              // MFC Ä£°åÀà¿â

//»ù´¡Êý¾Ý
#define PI       3.1415926         //Ô²ÖÜÂÊ
#define MAX_VALUE   1E30           //¼«´óÖµ
#define MIN_VALUE   1E-30          //¼«Ð¡Öµ
#define MAX_NUMB    30000          //×î´ó¼ÇÂ¼Êý
#define MAXZONE     20             //×î´ó·ÖÇøÊý

//=================================================================
//===================  Í¨ÓÃÀà¿â  ==================================
//=================================================================

//ÕûÊýÏµÁÐ,ÐòÁÐ±àºÅÈ«²¿´Ó0¿ªÊ¼
typedef CArray<int,int&> CIntArray;
//ÊµÊýÐòÁÐ,ÐòÁÐ±àºÅÈ«²¿´Ó0¿ªÊ¼
typedef CArray<float,float&> CValueArray;

//¿ÉÊÍ·ÅÄÚ´æµÄ¶ÔÏóÊý×é
class CPArray : public CObArray
{
public:
	  void RemoveAt(int index,int nCount=1);
	  void RemoveAll();
};

// 2 Î¬ÏòÁ¿  
struct FF_VECTOR                    
{
  float x;                      
  float y; 
};
typedef struct FF_VECTOR FF;
typedef CArray<FF,FF&> F2Array;
// 3 Î¬ÏòÁ¿ 
struct FFF_VECTOR                    
{
  float x;                      
  float y; 
  float xy;
};
typedef struct FFF_VECTOR FFF;
typedef CArray<FFF,FFF&> F3Array;

//ÕýÕûÊý¾ØÕóÔªËØ, ÎªÖ¸Ê¾¾ØÕóµÈ·þÎñ
class ILp:public CObject
{
public:
	CUIntArray pi;
    ILp();
    ILp(int n);
	~ILp();
    void Init(int n);
	void Reset();
    //¸ø³öÖ¸ÕëµÄÊýÄ¿
	int GetNp(){return pi.GetSize();}
    //ÅÐ¶ÏÄÄ¸öÎ»ÖÃº¬ÓÐÖ¸Õëpointer
    int LpWith(UINT pointer);
    //¸ø³öµÚloc¸öÎ»ÖÃµÄÖ¸Õëpointer
	int GetPointerOf(int loc);
};
//ÊµÊý¾ØÕóÔªËØ£¬ÎªÉøÁ÷¾ØÕóµÈ·þÎñ 
class FLp:public CObject       
{
public:
  CValueArray CLp;
  FLp();
  FLp(int n);
  ~FLp();
  void Init(int n);
  void Reset();
  //¸ø³öÊý¾ÝµãµÄÊýÄ¿
  int  GetNp(){return CLp.GetSize();}
  //¸ø³öµÚloc¸öÊý¾ÝµãµÄÊý¾Ý
  float GetValueOf(int loc);
};

//É¢µã¶¨ÒåÇúÏß
class FCurve:public CObject
{
public:
   CString mName; //ÇúÏßµÄÃû³Æ
   F2Array mp;    //É¢µã×ø±ê
public:
   FCurve();
   DECLARE_SERIAL(FCurve)
   FCurve(int n);
   virtual ~FCurve();
   
   //Çå¿ÕÐòÁÐ
   void Empty();
   //É¢µãµÄÊýÄ¿
   int  NumberOfNode(){return mp.GetSize();};
   //³õÊ¼»¯ÐòÁÐ
   void Init(int n);
   void Init(float y);
   void Init(float x1,float x2,float y1,float y2);
   void Init(int n,FF p0,FF p1);
   //²åÈëÒ»¸öÐÂµÄµã
   void InsertPoint(FF p);
   //¿½±´Êý¾Ý
   void Copy(const FCurve& src);
   //Y×ø±êÊý¾ÝÌáÈ¡
   BOOL  GetY_P(FF *p);
   float GetY_P(float x);
   //X×ø±êÊý¾ÝÌáÈ¡
   BOOL  GetX_P(FF *p);
   float GetX_P(float y);
   //Ð±ÂÊÌáÈ¡
   float GetSlope_X(float x,float sx_min,float sx_max);
   //¸´ÖÆÇúÏßÒ»¶Îµ½fNewÖÐ,ÓÉx1,x2È·¶¨¸´ÖÆ¶ÎµÄ³¤¶È
   BOOL  PartCopy(FCurve *fNew,float x1,float x2,BOOL reZero=FALSE);
   //¼ÆËãÇúÏßµÄÈ«¶Î»ý·Ö
   float Y_Integral();
   //±£´æ»ý·ÖÇúÏß
   void  Y_IntegralTo(FCurve *pCurve);  //½«»ý·ÖÇúÏß±£´æµ½ÁíÒ»ÌõÇúÏß
   void  Y_IntegralSelf();              //½«»ý·ÖÇúÏß±£´æµ½×ÔÉí
   float Y_Integral(float x1,float x2); //¼ÆËãÇúÏßµÄ¾Ö²¿»ý·Ö
   float Y_Average(float x1,float x2);  //¼ÆËã»ý·ÖÆ½¾ùÖµ
   //¸ø³öºá×ø±êÎªx1ºÍx2Á½µãyÊýÖµµÄ²îDy=y2-y1
   float GetDy(float x1,float x2);
   //Ð´ÎÄ¼þ
   void WriteFile(CStdioFile *fp);
   //¶ÁÎÄ¼þ
   BOOL ReadFile(CStdioFile *fp);
   BOOL ReadFile(FILE *fp);
   //´®ÎÄ¼þ²Ù×÷
   void Serialize(CArchive &ar);
};

//É¢µãÇúÏßÊý¾Ý¿â
class CurveLib:public CObject
{
public:
   CPArray  mCvs;                   //ÇúÏßÐòÁÐ
public:
   CurveLib();
   ~CurveLib();
   //²éÑ¯ÇúÏßÊýÄ¿
   int  NumberOfCurve(){return mCvs.GetSize();};
   //Çå¿Õ
   void Empty();
   //ÌáÈ¡Ç¡µ±ÇúÏßµÄÊý¾Ý
   BOOL GetY_P(int indexCurve,FF *pXY);
   BOOL GetY_P(CString cName,FF *pXY);
   //ÌáÈ¡Dy=y2-y1
   float GetDy(int indexCurve,float x1,float x2);
   //Ôö¼ÓÒ»ÌõÇúÏß
   int  ImportCurve(const FCurve &cv);
   //²éÕÒÃû³ÆÎªcNameµÄÇúÏßµÄIndex
   int  GetIndexCurve(CString cName);
   //²éÕÒË÷ÒýÎªindexµÄÇúÏßÃû³Æ
   CString GetCurveName(int index);
   //Êä³öÒ»ÌõÇúÏß
   BOOL ExportCurve(FCurve *pCv,int index);
   int  ExportCurve(FCurve *pCv,CString cName);
   //É¾³ýÒ»ÌõÇúÏß
   BOOL DelCurve(CString cName);
   BOOL DelCurve(int index);
   //¸üÐÂÒ»ÌõÇúÏß
   BOOL RenewCurve(const FCurve &cv,int index=-1);
   //Ð´ÈëÊý¾Ý¿âÎÄ¼þ
   BOOL WriteLibFile(CString lpFilePath);
   //¶ÁÈ¡Êý¾Ý¿âÎÄ¼þ
   BOOL ReadLibFile(CString lpFilePath);
};

//ÑÝ±äµãÕóÀà
class VariableField_Float:public CObject
{
protected:
	int             nNode;          //µãÕóµÄµãÊý
    int             mRefTVD;        //È±Ê¡µÄ¶¯Ì¬ÇúÏßË÷Òý
	float           mRefValue;      //È±Ê¡µÄ·ÖÅäÏµÊý
public:
	BOOL            bDependFile;    //·ÖÅäÏµÊýÈ¡¾öÓÚÎÄ¼þ
	CString         lpFileDis;      //´æ·Å·ÖÅäÏµÊýµÄÎÄ¼þ, ¶þ½øÖÆÎÄ¼þ
	CurveLib       *pLibTVD;        //Ê±¼äÐòÁÐÇúÏßÊý¾Ý¿â
	CIntArray       mIndexTVD;      //Ã¿¸öµãÕóµ¥Ôª¶ÔÓ¦µÄÑÝ±äÇúÏß
	CValueArray     mDisValue;      //·ÖÅäÏµÊý
	//ÎªñîºÏ´¦Àí·þÎñ
	CIntArray       mCellNode;      //ÓëÆäËüµãÕóµ¥ÔªµÄ¶ÔÓ¦¹ØÏµ,ÀýÈçnode-1Óëcell-23¶ÔÓ¦,ÆäÖÐcell-23ÊÇÆäËüµãÕóµÄµÚ23¸öµ¥Ôª             
public:
    VariableField_Float();
    ~VariableField_Float();
	//»ñÈ¡µãÊý
    int NumberOfNode(){return nNode;}
	//³õÊ¼»¯
	void SetSize(int n_node);
	void Init(int n_node, BOOL bVarDef=FALSE, BOOL bDisDef=FALSE);
	void Copy(VariableField_Float *pVF);
	//»ñÈ¡ºÍÉèÖÃÈ±Ê¡Êý¾Ý
	float GetRefValue();
	void  SetRefValue(float v);
	int   GetRefIndexTVD();
	void  SetRefIndexTVD(int index);
	//ÐÎ³ÉÒ»¸öºã¶¨¾ùÔÈµÄÊý¾ÝµãÕó
	void Reset_ConstantField(int n_node, float value);
	//ÐÎ³ÉÒ»¸ö¾ùÔÈµÄÑÝ±äÊý¾ÝµãÕó
    void Reset_UniformField(int n_node, const FCurve &DepCv);
	//É¾³ýÊý¾Ý
    void Empty();
	//´Ó·ÖÅäÏµÊýÎÄ¼þÖÐ¶ÁÈ¡·ÖÅäÏµÊý
	BOOL LoadDisValue();
	//ÎÄ¼þ¶ÁÐ´
	BOOL FileExchange(CStdioFile *fp, BOOL bRead=TRUE);
	//¾ßÓÐÇúÏßÊý¾Ý¿â¹ØÁªµÄÎÄ¼þ¶ÁÐ´
    BOOL FileExchange_LibLink(CStdioFile *fp, CurveLib *pLib, BOOL bRead=TRUE);
	//ÐÎ³ÉÒ»¸öÌØ¶¨Ê±¼äµÄµãÕó, dt>0 ±íÊ¾Ê±¼äÐòÁÐÎªÀÛ¼ÆÖµ
	BOOL OnTimeField(CValueArray *pField, float time, float dt=0);
	//°ÑÌØ¶¨Ê±¼äµÄµãÕóÊý¾Ý¼ÓÈëµ½Íâ²¿Á¬½ÓµÄµãÕóÊý¾Ý
	BOOL OnTimeLink(CValueArray *pFieldLink, float time, float dt=0);
};

//================================================================
//================   È«¾Öº¯Êý  ===================================
//================================================================

//¸ø³öÁ½×éÊýÖµµÄ×î´ó²î±ðµÄ¾ø¶ÔÖµ
float MaxDifference(CValueArray *pA,CValueArray *pB);
//¼ÆËãÁ½µãµÄ¾àÀë
float Distance(FF p1,FF p2);
//ÓëBFDÎÄ¼þµÄÊý¾Ý½»»», ¶þ½øÖÆµÄµãÕóÊý¾ÝÎÄ¼þ
int  ExchangeBFD(CString lpFile,CValueArray *pDat,BOOL Read=TRUE);
//ÓëD2DÎÄ¼þµÄÊý¾Ý½»»»
BOOL ExchangeD2D(CString lpFile,CPoint *pN,CValueArray *pDat,BOOL Read=TRUE);
//ÓëD3DÎÄ¼þµÄÊý¾Ý½»»», ±êÁ¿³¡, ¾ØÐÎÍø¸ñ nLay<=0 Îª¶ÁÈ¡Êý¾Ý, ·ñÔòÐ´Èë
int  ExchangeD3D(CString lpFile,CPoint *pN,CValueArray *pDat,int nLay=0);
//Çó½â¶Ô½Ç·½³Ì×é
//a*x[i-1]+b*x[i]+c*x[i+1]=d
//a[0]=0;c[Np-1]=0.
int tridag(int Np,float *a,float *b,float *c,float *d,float *x);

//=================================================================
//================ Ä£ÐÍ¿ØÖÆºÍÇó½â Àà¿â  ===========================
//=================================================================

//×ÜÌåÄ£ÐÍ¿ØÖÆ
class ModelCtrl:public CObject
{
public:
	CValueArray mDtSerial;     //Ê±¼ä²½³¤ÐòÁÐ
//·½³Ì×éÇó½â²ÎÊý
	BOOL        mEqLay;        //ÊÇ·ñ·Ö²ãµü´ú·¨(Ö»ÓÃÓÚÈýÎ¬µØÏÂË®Ä£ÐÍ)
	int         mMaxCount;     //×î´óµü´ú´ÎÊý
	float       mMaxDf;        //µü´ú¾«¶È
	float       mMaxChange;    //µü´ú½á¹û±ä»¯µÄ¿ØÖÆÉÏÏÞ
	float       mC_Sor;        //µü´úËÉ³ÚÒò×Ó
//·ÇÏßÐÔ´¦Àí²ÎÊý
	float       m_cNormal;     //ÖÐ¼ä×´Ì¬¼ÓÈ¨Òò×Ó
	int         mItCpl;        //×î´ó·ÇÏßÐÔµü´ú´ÎÊý
//Êä³ö¿ØÖÆ
	int         mCountPrint;   //½á¹ûÊä³öµÄÊ±¼ä²½³¤¼ä¸ô

public:
    ModelCtrl();
    ~ModelCtrl();
	void  Copy(ModelCtrl *pCtrl);
	float GetTotalTime();                  //×Ü¹ý³ÌÊ±¼ä
    void  SetUniformDt(float dt,int nDt);  //¾ùÔÈµÄÊ±¼ä²½³¤ÐòÁÐ
	//ÉèÖÃ¼ÓËÙ»ò¼õËÙµÄÊ±¼ä²½³¤ÐòÁÐ
    void  SetVariableDt(int nDt,float dt0,float cdt);
	//¶ÁÈ¡ºÍÐ´ÈëÄ£ÐÍ¿ØÖÆÐÅÏ¢ÎÄ¼þ
	BOOL  ToFormatFile(CString lpFile,BOOL onSave=TRUE);
};

// Ä£ÐÍÔËÐÐ×´Ì¬ÐÅÏ¢ 
class ModelRunInfo                    
{
public:
  int      mStepIndex;            //Ê±²½
  int      mStepMax;              //×î´óÊ±²½Êý
  int      mSP_Count;             //ÌØ¶¨¹ý³Ì´¦Àí´ÎÊý
  int      mSP_MAX;               //ÌØ¶¨¹ý³Ì´¦Àí×î´ó´ÎÊý
  int      mEQ_ItCount;           //·½³Ì×éÇó½âµü´ú´ÎÊý
  float    mDfMax;                //Ò»´Îµü´úµÄ×î´ó±ä»¯·ù¶È
  float    mStMax;                //×´Ì¬±ä»¯·ù¶È
  CString  mStat;                 //¹ý³ÌÃèÊö
};

//Ò»Î¬ÏßÐÔ·½³Ì×é [CLp]*X=Q
class EqGroup:public CObject
{
protected:
	BOOL    isInit;   //ÊÇ·ñÒÑ¾­³õÊ¼»¯
public:
    CPArray mILp;     //Ö¸Ê¾¾ØÕó, ILp
	CPArray mCLp;     //ÏµÊý¾ØÕó, FLp
    CValueArray mQ;   //ÓÒ¶ËÏòÁ¿
	CValueArray *mX;  //Î´ÖªÏòÁ¿
public:
	ModelRunInfo *pInfo;
	EqGroup();
	~EqGroup();
//ÊÇ·ñ³õÊ¼»¯
	BOOL IsInit(){return isInit;}
//¼ì²âÓÐÐ§ÐÔ
	BOOL IsValid();
//²ÉÓÃËÉ³Úµü´ú·¨Çó½â·½³Ì×é, ·µ»Øµü´úÎó²î
    float SolveEq(int nMax,float mDf,float C_Sor,ModelRunInfo *pRunInfo=NULL,float CtrlChange=0);
};

//======================================================================
//==================== ÈýÎ¬¾ØÐÎÓÐÏÞ²î·ÖÍø¸ñ: ÀàºÍº¯Êý  =================
//======================================================================

#define GRID_V   -1
#define GRID_X   0                 // X×ø±êÖá·½Ïò
#define GRID_Y   1                 // Y×ø±êÖá·½Ïò
#define GRID_Z   2                 // Z×ø±êÖá·½Ïò

#define SIDE_W   0                 // Î÷²à
#define SIDE_N   1                 // ±±²à
#define SIDE_E   2                 // ¶«²à
#define SIDE_S   3                 // ÄÏ²à
#define SIDE_U   4                 // ÉÏ²à(¶¥²à)
#define SIDE_L   5                 // ÏÂ²à(µ×²à)

#define MAX_LAYERS  200            //Íø¸ñ²ãµÄ×î´óÊýÄ¿

//µ¥ÔªµÄÈ«¾Ö±àºÅ´Ó0¿ªÊ¼
class Grid_3D:public CObject
{
public:
   //Z·½ÏòµÄ¸ñ×ÓÊý, ´Ó¶¥²¿¿ªÊ¼¼ÆÊý
   int         mNLayer;               //Íø¸ñ²ãÊý
   CValueArray mDx;                   //X·½Ïòµ¥Ôª¸ñµÄ¿í¶È
   CValueArray mDy;                   //Y·½Ïòµ¥Ôª¸ñµÄ¿í¶È
   CValueArray mZ0;                   //ÈýÎ¬Íø¸ñ¶¥²¿µÄ×ø±ê£¬¼´µÚ0²ãµÄ¶¥²¿¸ß¶È,Ny*Nx
   CValueArray mDzGrid[MAX_LAYERS];   //ÈýÎ¬Íø¸ñµÄ¸÷²ãµ¥ÔªµÄºñ¶È, Nz*Ny*Nx
   //ÎïÀí×ø±êÐÅÏ¢
   float       mX0;                   //Ë®Æ½Ô­Ê¼×ø±ê
   float       mY0;  
   float       mAngle;                //ÄæÊ±ÕëÐý×ªµÄ½Ç¶È
public:
   Grid_3D();
   DECLARE_SERIAL(Grid_3D)
   virtual ~Grid_3D();

public:
//»ù±¾ÉèÖÃºÍ¼¸ºÎÐÅÏ¢ÌáÈ¡
   void RemoveGrid();
   //³õÊ¼»¯¾ùÔÈÍø¸ñ
   void InitGeometry(int Nx,int Ny,int Nz,float dx,float dy,float dz,float z0=0);
   //ÖØÐÂÉèÖÃÍø¸ñµÄ¼¸ºÎÐÎ×´
   void ResetGeometry(int Nz,CValueArray *pDx,CValueArray *pDy,CValueArray *pZ0,CValueArray **pDzGrid);
   void ResetGeometry(CValueArray *pDx,CValueArray *pDy,CValueArray *pDzLayer,float z0=0); 
   //¼ì²âÍø¸ñÊÇ·ñÓÐÐ§
   BOOL CheckGrid();
   //µ÷Õûdx,dy
   BOOL ResetDxDy(int index,float newDL,int Dir);
   //µ÷ÕûÄ³¸öµ¥ÔªÎ»ÖÃµÄ¶¥²¿¸ß¶È
   BOOL ResetZ0(int ix,int iy,float newZ0);
   //µ÷ÕûÄ³¸öµ¥ÔªµÄºñ¶È
   BOOL ResetDz(int ix,int iy,int iz,float newDZ);
   //µ÷ÕûÄ³²ãµ¥ÔªµÄºñ¶È
   BOOL ResetDz(int iz,float newDZ);
   //¸ù¾ÝÈýÎ¬Ë÷ÒýtIndex¸ø³öÈý¸ö·½ÏòµÄË÷Òý
   int GetCellLayer(CPoint *pNode,int tIndex);
   //»ñµÃµ¥ÔªµÄÈýÎ¬Ë÷Òý
   int GetCellIndex(int ix,int iy,int iz);
   //»ñµÃµ¥ÔªµÄÆ½ÃæË÷Òý
   int GetCellIndex(int ix,int iy);
   //»ñµÃÏàÁÚµ¥ÔªµÄÈýÎ¬Ë÷Òý
   int GetCellNeighbor(int ix,int iy,int iz, int side);
   //¸ù¾ÝË÷Òý¸ø³öÄ¿±ê²ãÎ»
   int GetCellObLayer(int index);
   //¸ù¾ÝË÷Òý¸ø³öix,iy
   BOOL GetCellRowCol(CPoint *pRC,int index);
   //»ñµÃÄ³¸ö·½ÏòµÄµ¥Ôª¸ñÊýÄ¿,Direction=GRID_X, GRID_Y, GRID_Z
   int GetCellNumber(int Direction);  
   //»ñµÃÈ«²¿µ¥Ôª¸ñÊýÄ¿
   int GetCellNumber();  
   //»ñµÃµ¥ÔªÎ»ÖÃµÄ¶¥²¿¸ß¶È
   float GetZ0Cell(int ix,int iy);
   //»ñµÃµ¥ÔªµÄºñ¶È
   float GetDZCell(int ix,int iy,int iz);
   //»ñµÃµ¥Ôª¸ñË®Æ½¿í¶È,Direction=GRID_X, GRID_Y
   float GetCellWidth(int index,int Direction);
   //¼ÆËãÕû¸öÍø¸ñµÄË®Æ½³ß´ç
   float GetGridWidth(int Direction);
   //¼ÆËãµ¥ÔªµÄÉÏ±íÃæ»ý
   float GetCellArea(int ix,int iy);
   float GetCellArea(int index);
   //»ñµÃÄ³¸öµ¥Ôª¸ñµÄ¼¸ºÎ³ß´ç±£´æÔÚdLÖÐdL[0]=dx,dL[1]=dy,dL[2]=dz
   BOOL GetCellGeometry(float *dL,int ix,int iy,int iz);
   //»ñµÃÄ³¸öµ¥Ôª¸ñµÄ¼¸ºÎ³ß´ç±£´æÔÚdLÖÐdL.x=dx,dL.y=dy,dL.xy=dz
   BOOL GetCellGeometry(FFF *dL, int ix, int iy, int iz);
   //»ñµÃÄ³¸öµ¥ÔªµÄÐÍÐÄË®Æ½×ø±ê
   BOOL GetCellCenter(FF *pCent,int index_x,int index_y,BOOL isCircum);
   //»ñµÃÄ³¸öµ¥Ôª¸ñÈýÎ¬ÐÍÐÄ×ø±êpC_x;pC_y;pC_xy
   BOOL GetCellCenter(FFF *pC, int ix, int iy,int iz,BOOL isCircum);
   BOOL GetCellCenter(FFF *pC, int index,BOOL isCircum);
   //»ñµÃÄ³¸öµ¥Ôª¸ñµÄµ×²¿Z×ø±ê
   float GetCellBottom(int ix,int iy,int iz);
   //»ñµÃÄ³ÐÐ¡¢ÁÐµÄ×î¸ß¶¥²¿¸ß¶È
   float GetMaxZ0(int sel,int dir);
   //»ñµÃÄ³ÐÐ¡¢ÁÐµÄ×îµÍµ×²¿¸ß¶È
   float GetMinBottom(int sel,int dir);

//Êý¾Ý½»»»ºÍ´¦Àí
public:
	//ÎÄ¼þ²Ù×÷
	void Serialize(CArchive& ar);
    //ÎÄ±¾¸ñÊ½ÎÄ¼þ×ª»»
    BOOL ToFormatFile(CString filePath, BOOL isSave);
	//´ÓÐÅÏ¢ÎÄ¼þÖÐ¶ÁÈ¡¸ñÊ½»¯µÄÈýÎ¬Íø¸ñÊý¾Ý
    BOOL ReadDataD3D(CStdioFile *fp,CValueArray *pDat);
	//µ¼ÈëÊý¾Ýµ½Ö¸¶¨µÄÊý×é£¬ÓëSurferµÄGRDÎÄ¼þ½»»»Êý¾Ý
  	BOOL ImportGRD(CValueArray *pData,CString fname);
    //Êä³öD2D¸ñÊ½²ãÊý¾ÝÎªCSVÎÄ¼þ
	BOOL D2D_DataCSV(CValueArray *pData,CString fname);
	//Êä³öD3D¸ñÊ½Êý¾ÝpDataµÄÄ³Ò»²ãÊý¾ÝÎªCSVÎÄ¼þ
    BOOL D3D_LayDataCSV(int lay,CValueArray *pData,CString fname);
	//Êä³öD3D¸ñÊ½Êý¾ÝpDataµÄX·½ÏòÆÊÃæ(iy)Êý¾ÝÎªCSVÎÄ¼þ
    BOOL D3D_LatiDataCSV(int iy,CValueArray *pData,CString fname);
	//Êä³öD3D¸ñÊ½Êý¾ÝpDataµÄY·½ÏòÆÊÃæ(ix)Êý¾ÝÎªCSVÎÄ¼þ
    BOOL D3D_LongiDataCSV(int ix,CValueArray *pData,CString fname);
	//Êä³öD3D¸ñÊ½Êý¾ÝpDataµÄÇÐÃæÊý¾ÝÎªCSVÎÄ¼þ, dirÊÇÇÐÃæ·¨Ïß·½Ïò
    BOOL D3D_CrossDataCSV(int cross,int dir,CValueArray *pData,CString fname);
	//¶ÔÒ»²ãÍø¸ñµÄÕûÐÍÊý¾Ý¸³Öµ
	BOOL ResetValueInt(int value,CIntArray *pGrdDat,int lay=-1);
	//¶ÔÒ»²ãÍø¸ñµÄÊµÊýÐÍÊý¾Ý¸³Öµ
	BOOL ResetValueFloat(float value,CValueArray *pGrdDat,int lay=-1);

//ÆäËü¸¨Öúº¯Êý
public:
   //¸ø³öÒ»¸öÈ«¾ÖÎïÀí×ø±êµãµÄÄ£ÐÍ¾Ö²¿×ø±ê
   void LocalPlace(FF *pLoc,FF *pGlob,BOOL CircumVolve=FALSE);
   //È·¶¨thePointÔÚÄÄ¸öÆ½ÃæÍø¸ñµ¥ÔªÄÚ, Èç¹û²»ÔÚÍø¸ñÇø, Ôò·µ»ØFALSE
   BOOL GetCell_XY(CPoint *pCell,FF *pXY,BOOL CircumVolve=FALSE);
   //È·¶¨Æ½ÃæÉÏÒ»ÌõÖ±ÏßÓëÍø¸ñÏßµÄ½»µãÐòÁÐ, °´ÕÕ¾àÀëp1ÓÉÐ¡µ½´óË³ÐòÅÅÁÐ
   void LineWithGrid_XY(F2Array *pLine,FF p1,FF p2,BOOL CircumVolve=FALSE);
};

//========================================================
//                  º¬Ë®²ãÄ£ÐÍ»ù´¡Àà
//========================================================

//º¬Ë®²ãµ¥Ôª²ÎÊý¼¯
struct aPamAquifer
{
    float  Ksx;     //±¥ºÍÉøÍ¸ÏµÊýKsx
	float  Ksy;     //±¥ºÍÉøÍ¸ÏµÊýKhy
    float  Ksv;     //±¥ºÍÉøÍ¸ÏµÊýKv
    float  Ss;      //±¥ºÍ, ÖüË®ÂÊ
	float  nVoid;   //½éÖÊ×Ü¿×Ï¶¶È
	float  rVoid;   //½éÖÊ²ÐÓà¿×Ï¶¶È
	float  Ck;      //·Ç±¥ºÍµ¥ÔªµÄÉøÍ¸ÐÔË¥¼õ³£Êý
	float  Cw;      //·Ç±¥ºÍµ¥ÔªµÄº¬Ë®Á¿Ë¥¼õ³£Êý
	int    iKr;     //Ïà¶ÔÉøÍ¸ÏµÊý±ä»¯ÇúÏßµÄË÷Òý
    int    iSe;     //ÓÐÐ§±¥ºÍ¶È±ä»¯ÇúÏßµÄË÷Òý
};
typedef struct aPamAquifer APam;
typedef CArray<APam,APam&> APamArray;  //²ÎÊýÐòÁÐ

//º¬Ë®²ã-¿×Ï¶Àà
class Aq_Pore:public CObject    
{
public:
  Aq_Pore();
  ~Aq_Pore();
  APam       mPam;        //µ¥Ôª²ÎÊý¼¯
  CurveLib  *pLibCv;      //ÇúÏßÊý¾Ý¿âÖ¸Õë
public:
  //ÀûÓÃvan Genuchten ¹«Ê½ÐÎ³ÉÓÐÐ§±¥ºÍ¶È±ä»¯ÇúÏß
  BOOL  VG_HydrCurve(FCurve *pCv,float a, float n);
  //¸ù¾Ý»ùÖÊÎüÁ¦hp¸ø³öÓÐÐ§±¥ºÍ¶È
  float GetSeDefault(float hp);
  float GetSe(float hp);  
  //¸ù¾Ý»ùÖÊÎüÁ¦hp¸ø³öÏà¶ÔÉøÍ¸ÏµÊý
  float GetKrDefault(float hp);
  float GetKr(float hp);
  //¸ù¾Ý»ùÖÊÎüÁ¦hp¸ø³öÌå»ýº¬Ë®Á¿
  float GetWaterContent(float hp);
  //¸ù¾Ý»ùÖÊÎüÁ¦hp¸ø³ö·Ç±¥ºÍÈÝË®¶È
  float GetWaterCapacity(float hp);
  //¸ù¾Ý»ùÖÊÎüÁ¦h1, h2¸ø³öÏà¶ÔÉøÍ¸ÏµÊýµÄ»ý·ÖÆ½¾ùÖµ
  float Kr_MeanDefault(float h1,float h2);
  float Kr_Mean(float h1,float h2);
  //¸ù¾Ý»ùÖÊÎüÁ¦h1, h2¸ø³öÖüË®ÂÊµÄ»ý·ÖÆ½¾ùÖµ
  float Ss_Mean(float h1,float h2);
};

//ÄÚÖÃ±ß½ç²ÎÊý¼¯
//ÎªÁËÊÊÓ¦¸÷ÖÖÀàÐÍµÄ±ß½ç
//QtÎªÍ¨Á¿Ç¿¶È£¬ÎªÊµ¼ÊÁ÷Á¿, ÒÔ²¹¸øº¬Ë®²ãÎªÕý
// ( 1--> Ht; 2---> Qt=Cq*Qb(t)+Ch*(Ht-H)=Bt-Ch*H, m/d; )
struct aInBound
{
	BOOL   isFlux;  //ÊÇ·ñÎª±äÁ÷Á¿±ß½ç,²¹¸øÎªÕý
	float  Ch;      //¹ØÁªÏµÊý
    float  refBt;   //±ß½ç²ÎÊýÈ±Ê¡Öµ
	int    pTimeBt; //¶¯Ì¬ÇúÏßµÄÖ¸ÕëºÅ
};
typedef struct aInBound InBound;
typedef CArray<InBound,InBound&> InBoundArray;   //ÄÚÖÃ±ß½çÐòÁÐ

//¹ØÁªÏµÊý
//        CE(hE-h)+CW(hW-h)+CN(hN-h)+CS(hS-h)
//       +CU(hU-h)+CL(hD-h)
//         +CC*h+Qc          //ÓëÖÜÎ§µ¥ÔªÎÞ¹ØµÄÏµÊýºÍÔ´»ãÏî
//      = SV(h-h0)/dt
struct _CSeep
{
	float CN;  //North Side
    float CE;  //East  Side
	float CS;  //South Side
	float CW;  //West  Side
	float CU;  //Upper Side
	float CL;  //Lower Side  
	float SV;  //Storage Volumme
	float CC;  //ÖÐÐÄÏµÊý
	float Qc;  //³£ÊýÔ´»ãÏî
};
typedef struct _CSeep CSeep;
typedef CArray<CSeep,CSeep&> CSeepArray;   //¹ØÁªÏµÊýÐòÁÐ

//================================================================
//================   º¬Ë®²ã-Ë®ÎÄÄ£¿éñîºÏ»ùÀà  ====================
//================================================================

class AquiferFlow;                     //º¬Ë®²ãÄ£ÐÍÀà

class AquiferHydro_Module
{
//========  ÊôÐÔ =============
public:  
	AquiferFlow *pTheAquifer;          //º¬Ë®²ãË®Á÷Ä£ÐÍ
    BOOL         isActive;             //ÊÇ·ñ¼¤»î
//========  ¹¹Ôì  ============
public:
	AquiferHydro_Module();
	~AquiferHydro_Module();

//======== Ä£¿é²Ù×÷ ==========
public:
//ÎªÔËÐÐ×ö×¼±¸
    virtual BOOL Run_Prepare();
//±¸·Ý³õÊ¼Ìõ¼þ
    virtual BOOL SaveInitialCondition(BOOL onSave=TRUE);
//¶Ô±ÈÁ½´Îµü´ú½á¹û
	virtual float CompareIteration(BOOL justSave=TRUE);
//¸üÐÂ×´Ì¬
	virtual BOOL StatusChange(float t1,float t2);
//¹Ì¶¨×´Ì¬µÄÔËÐÐÒ»²½
   	virtual int  StepFlow_Fixed(float t1,float t2);
//¸ù¾Ý³õÊ¼Ìõ¼þÐÎ³ÉÖÐ¼ä×´Ì¬
    virtual BOOL ResetNormalCondition(float cNormal=1.0f);
//Ç°½øÒ»²½
	virtual BOOL StepForward(float time=0,BOOL onWriteObs=FALSE,BOOL onWriteOut=FALSE);
//¹Ø±Õ
	virtual BOOL Close();
//======== ñîºÏ²Ù×÷  ==========
public:     
//½»»»Êý¾Ý
    virtual BOOL ExchangeData();   
//×´Ì¬¾ØÕóµÄÏà»¥×÷ÓÃ
	//×÷ÓÃµ½º¬Ë®²ã
    virtual BOOL StatusOnAquifer(float t1,float t2);
	//×÷ÓÃµ½Ä£¿é
    virtual BOOL StatusOnModule(float t1,float t2);
};

//================================================================
//================  º¬Ë®²ãµØÏÂË®Á÷Ä£ÐÍ   =============================
//================================================================

//µ¥ÔªÊôÐÔ¿ØÖÆ, ID>0±íÊ¾¸÷ÖÖ²»Í¬µÄÄÚÖÃ±ß½çÀàÐÍ
#define CELL_INACTIVE  -999    //ÎÞÐ§µ¥Ôª
#define CELL_CONHEAD   -998    //¹Ì¶¨Ë®Í·µ¥Ôª, Õû¸öÄ£ÄâÊ±ÆÚ¹Ì¶¨
#define CELL_KNOWNHEAD -997    //ÒÑÖªË®Í·µ¥Ôª, Ä³¸öÊ±²½ÄÚ¹Ì¶¨
#define CELL_ACTIVE    0       //³£¹æÓÐÐ§µ¥Ôª
//²ÎÊýÖ¸Ê¾ÐòÁÐ
#define PORE_KSX       1       //±¥ºÍÉøÍ¸ÏµÊýKsx
#define PORE_KSY       2       //±¥ºÍÉøÍ¸ÏµÊýKhy
#define PORE_KSV       3       //±¥ºÍÉøÍ¸ÏµÊýKsz
#define PORE_SS        4       //±¥ºÍÖüË®ÂÊ
#define PORE_NVOID     5       //½éÖÊ×Ü¿×Ï¶¶È
#define PORE_RVOID     6       //½éÖÊ²ÐÓà¿×Ï¶¶È
#define PORE_CK        7       //·Ç±¥ºÍµ¥ÔªµÄÉøÍ¸ÐÔË¥¼õ³£Êý
#define PORE_CW        8       //·Ç±¥ºÍµ¥ÔªµÄº¬Ë®Á¿Ë¥¼õ³£Êý
#define PORE_IKR       9       //Ïà¶ÔÉøÍ¸ÏµÊý±ä»¯ÇúÏßµÄË÷Òý
#define PORE_ISE       10      //ÓÐÐ§±¥ºÍ¶È±ä»¯ÇúÏßµÄË÷Òý
#define CELL_IDFLOW    11      //µ¥ÔªµÄÊôÐÔÖµ
//================================================================
//================   AquiferFlow  ================================
//================================================================
class AquiferFlow:public CObject
{
public:
    Grid_3D     *mGrid;            //ÈýÎ¬Íø¸ñ
    APamArray    mPam;             //Ë®ÎÄµØÖÊ²ÎÊý
    ModelCtrl    mCtrl;            //Ä£ÐÍ¿ØÖÆ

    CurveLib    *pLibCurve;        //ÇúÏßÊý¾Ý¿â, ´æ·ÅÓÐ·Ç±¥ºÍÌØÕ÷ÇúÏßµÈ
   
    InBoundArray mBound;           //¶àÖÖÄÚÖÃ±ß½ç
	//µ¥ÔªÊôÐÔ
    //Èç¹û mID_FLOW=CELL_INACTIVE, ÎªÎÞÐ§µ¥Ôª
	//Èç¹û mID_FLOW=CELL_CONHEAD, Îª¹Ì¶¨Ë®Í·µ¥Ôª
    //Èç¹û mID_FLOW=CELL_KNOWNHEAD=-997, ÎªÔÝÊ±ÒÑÖªË®Í·µ¥Ôª
	//Èç¹û mID_FLOW>=0, Ôò
	//     mID_FLOW=0 Îª³£¹æÓÐÐ§µ¥Ôª,²»Á¬½ÓÈÎºÎ±ß½ç²ÎÊý
	//     -900<mID_FLOW<0 ÎªÆäËüÓÐÐ§µ¥Ôª
	//     µ±mID_Flow>0Ê±
	//     ID=mID_FLOW-1 ÎªÓÐÐ§ÄÚÖÃ±ß½çÀàÐÍºÅ
	CIntArray    mID_FLOW;         //µ¥ÔªÉøÁ÷±ß½çÊôÐÔ
	
    CValueArray  mZbCell;          //Ã¿¸öµ¥ÔªµÄµ×²¿¸ß¶È

    CValueArray  mH0;              //³õÊ¼Ë®Í·
    CValueArray  mH;               //µ±Ç°Ë®Í·
    CValueArray  mHb;              //±¸·ÝË®Í·

	CSeepArray   mCSeep;           //ÉøÁ÷×´Ì¬¾ØÕó
	EqGroup      mEq;              //ÓÐÏÞ²î·Ö·½³Ì×é

	//ÌØÊâÌõ¼þ´¦Àí
	CUIntArray   mObs;             //¹Û²ìµãµ¥Ôª 
	float        mMinChange;       //×´Ì¬¸üÐÂµÄ×îÐ¡Ë®Í·±ä»¯ 
	BOOL         mTopConfined;     //Ä£ÐÍ¶¥²¿Îª·â±ÕµÄ, ·ñÔòÎªµØ±í
 
    //Ä£ÐÍÏîÄ¿ÎÄ¼þ
	CString     lpFileInf;         //°üº¬ÅäÖÃÐÅÏ¢µÄÎÄ¼þ
	CString     lpProject;         //Ä£ÐÍµÄÃû³Æ
    CString     lpOutFile;         //ÕóÁÐ½á¹ûÊä³öÎÄ¼þ
	CString     lpObsFile;         //¹Û²ìµã½á¹ûÊä³öÎÄ¼þ
	CString     lpScanFile;        //¿ìËÙÉ¨ÃèÎÄ¼þ
    CStdioFile  fpScan,fpObs,fpOut;

	//ÎÄ¼þÉ¨Ãè¼ÇÂ¼±äÁ¿
    CValueArray timeObs,timeOut;
    CDWordArray localObs,localOut;
    DWORD       pObs,pOut,longObs,longOut;

	//ÔËÐÐ¹ý³ÌÐÅÏ¢
    ModelRunInfo *pRunInfo;         

public:
    AquiferFlow();
	~AquiferFlow();
    AquiferFlow(Grid_3D *gd);
//´´½¨Ä£ÐÍ
	void Create(Grid_3D *gd,float h0=0);
//Çå³ýÄ£ÐÍµÄËùÓÐÊý¾Ý
	void RemoveData();
//¶ÁÈ¡Ä£ÐÍÅäÖÃÎÄ¼þ
	BOOL ReadModelSetting(CString lpSetFile);
//¶ÁÈ¡¹Û²ìµãµ¥ÔªÎ»ÖÃÎÄ¼þ
	BOOL ReadObsCell(CString lpFileInf);
//¶ÁÈ¡ÄÚÖÃ±ß½çÐÅÏ¢ÎÄ¼þ
    BOOL ReadInBound(CString lpFileInf);
//»ñÈ¡µ¥ÔªµÄµ×²¿¸ß¶È
	BOOL GetCellBottom();
//Ôö¼ÓÒ»¸öÄÚÖÃ±ß½ç
	int  AddInBound(InBound aInBound);
//¼ì²âÄ£ÐÍ
	int  IsModelValid();
//Ð´É¨ÃèÎÄ¼þ\½á¹ûÎÄ¼þ\¹Û²ìÎÄ¼þµÄÍ·²¿
    BOOL WriteFileHead();
//Ð´¹Û²ìµã½á¹û
	BOOL WriteObsHead(float time);
//Ð´Íø¸ñË®Í·½á¹û
    BOOL WriteOutHead(float time);
//Ð´É¨ÃèÎÄ¼þ
	BOOL WriteScanFile();
//Ôö¼ÓÒ»¸ö¹Û²ìµã
	void InsetObsCell(int ix,int iy,int iz);
//ÉèÖÃÄ³¸öµ¥ÔªµÄË®Á÷±ß½çÊôÐÔ
	BOOL SetFlowType(int ix,int iy,int iz,int idFlow);
//ÅÐ¶ÏÄ³¸öµ¥ÔªÊÇ·ñÎªÓÐÐ§µ¥Ôª
	int  IsActiveCell(int ix,int iy,int iz);
//ÅÐ¶ÏÄ³¸öµ¥ÔªÊÇ·ñÎªÒÑÖªË®Í·±ß½ç
	BOOL IsHeadBound(int ix,int iy,int iz);
//ÉèÖÃË®ÎÄµØÖÊ²ÎÊý
	BOOL InitPamSetting(APam aPam,int lay=-1);
//ÌáÈ¡Ë®ÎÄµØÖÊ²ÎÊý
	BOOL GetPamSetting(Aq_Pore *pore,int index);
	BOOL GetPamSetting(Aq_Pore *pore,int ix,int iy,int iz);
//²ÉÓÃ¾ØÕó·½Ê½ÊäÈëÄ£ÐÍµÄÌØ¶¨²ÎÊý
	BOOL ImportPamGrd(CValueArray *pGrdDat, int PamName);
//µ±Ç°Ë®Í·×´Ì¬ÏÂµÄ»ùÖÊÎüÁ¦ph->x=hp_top, ph->y=hp_bot, ph->xy=hp_c
    BOOL  GetHpCell(int ix,int iy,int iz, FFF *ph, CValueArray *pH);
//¸ù¾ÝË®Í·¼ÆËãµ¥ÔªµÄº¬Ë®Á¿
	float GetMoistureCell(int ix,int iy,int iz, float hCell);
//µ±Ç°Ë®Í·×´Ì¬ÏÂµÄË®Æ½µ¼Ë®ÏµÊý(pTxTy), ¶¥µ×²¿µÄ´¹ÏòÉøÍ¸ÏµÊý, ÖüË®ÏµÊý(·µ»ØÖµ)
	float GetTxTy_S(int ix,int iy,int iz,FF *pTxTy,FF *pKtKb);
	float GetKzTop(int ix,int iy);
//¼ÆËãËùÓÐµ¥ÔªµÄ×èË®ÏµÊýºÍÖüË®ÏµÊý
	BOOL  Resistance_Storage(F2Array *pRxy,F2Array *pRz,CValueArray *pS);
//Éú³ÉÉøÁ÷ÏµÊý¾ØÕó
	BOOL CSeep_Create(CValueArray *pH);
//¸ù¾ÝË®Í·±ä»¯ÐÞ¸ÄÉøÁ÷ÏµÊý¾ØÕó
    BOOL CSeep_ForChange(CValueArray *pH,CValueArray *pH_old,float minChange);
//¸ù¾ÝÄÚÖÃ±ß½çÌõ¼þ³õÊ¼»¯ÉøÁ÷ÏµÊý¾ØÕóµÄÔ´»ãÏî
    BOOL CSeep_ModifyInBound(float time);
//¸ù¾ÝÄÚÖÃ±ß½çÌõ¼þ³õÊ¼»¯´ýÇóµÄË®Í·ÐòÁÐ
	BOOL ResetHead_InBound(float time,CValueArray *pH);
//ÊÍ·ÅÔÝÊ±¶¨Ë®Í·±ß½ç
	void ReleaseTemp_KnownCell();
//³õÊ¼»¯´úÊý·½³Ì×é
    BOOL Eq_Init();
    BOOL EqLay_Init();  //·Ö²ãµü´ú·¨
//¸ù¾Ý×îÐÂµÄÉøÁ÷ÏµÊý¾ØÕó¸üÐÂ´úÊý·½³Ì×é
    BOOL Eq_ResetCSeep(float dt,CValueArray *h0);
    BOOL EqLay_ResetCSeep(float dt,CValueArray *h0); //·Ö²ãµü´ú·¨
//·Ö²ãµü´ú·¨¸üÐÂ·½³Ì×éÓÒ¶ËÏî
	float EqLay_mQ(float dt, float c_sor, CValueArray *h0, CValueArray *hp);
//ÎªÔËÐÐ×÷×¼±¸
	BOOL Run_Prepare();
//´¦Àí±¸·ÝË®Í· mHb
    BOOL StandbyInitialHead(BOOL onSave=TRUE);
//¶Ô±ÈÒ»´Îµü´ú¼ÆËã½á¹û
    float CompareIteration(BOOL justSave=TRUE);
//ÐÎ³ÉÖÐ¼ä×´Ì¬Ë®Í·
	BOOL SetNormalHead(float ch=1.0f);
//ÔËÐÐÒ»¸öÊ±¼ä²½³¤
	int  Run_StepFlow(float t1,float t2);
	void StepFlow_Prepare(float t1,float t2);  //×÷×¼±¸
	int  StepFlow_Fixed(float t1,float t2);    //¹Ì¶¨×´Ì¬·¨
	int  StepFlow_Varying(float t1,float t2);  //Ô¤²âÐ£Õý·¨
    int  StepFlow_Iteration(float t1,float t2);//²ÉÓÃµü´ú·¨
	BOOL StepForward(float time=0,BOOL onWriteObs=FALSE,BOOL onWriteOut=FALSE);
//ÔËÐÐÕû¸ö¹ý³Ì
	BOOL Run(ModelRunInfo *pRunInfo=NULL,BOOL endClose=TRUE);
//¹Ø±ÕÄ£ÐÍ
	void Close();

//===== ÒÔÏÂ±äÁ¿ÓÃÓÚÓëÆäËüÄ£¿éµÄ½Ó¿Ú==========================
public:
	AquiferHydro_Module  tempModule;          //ÁÙÊ±Ä£¿é
    AquiferHydro_Module *pModule[MAXZONE];    //Á¬½ÓÄ£¿é
//¼ÓÈëº¬Ë®²ã¶¥²¿µÄ²¹¸øÇ¿¶È
    BOOL Import_TopRecharge(CValueArray *pRechTop);
//¼ÓÈëÈýÎ¬·Ö²¼Ê½Ô´»ãÏî
	BOOL Import_SourceTerm(CValueArray *pSourceFlow);
//µØÏÂË®µÄË®Î»·Ö²¼
	BOOL ExportWaterTable(CValueArray *pWT);
//µØÏÂË®µÄÂñÉî·Ö²¼
    BOOL ExportTableDepth(CValueArray *pDep);
//º¬Ë®²ã¶¥²¿µÄ»ùÖÊÎüÁ¦ºÍ´¹ÏòÉøÍ¸ÏµÊý
	BOOL ExportHpKz_Top(CValueArray *ph,CValueArray *pKz=NULL);
//ÔÚÉøÁ÷ÏµÊý¾ØÕóÖÐÌí¼Ó¶¥²¿ÍÁÈÀÎüË®
    BOOL ImportUptaken_CSeep(CValueArray *pHSoil, CValueArray *pCLeak);
};

//=======================================================================
//=======================  º¬Ë®²ãÖ±½Ó¹ØÁªÄ£¿é  ==========================
//=======================================================================

//±äÇ¿¶È·Ö²¼Ê½Ô´»ãÏîÄ£¿é
//×¢ÒâmSourceÔ´Ç¿¶È(Á÷Á¿)µÄ¶¯Ì¬ÇúÏß¿ÉÄÜÊÇÀÛ¼ÆÇúÏß
class Aq_SinkSourceTerm:public AquiferHydro_Module
{
//========  ÊôÐÔ =============
public:  
	CString                 lpFileInf;            //°üº¬Ô´»ãÏîÐÅÏ¢µÄÎÄ¼þ
	//Ô´»ãÏîÊÇÒ»ÏµÁÐÑÝ±äµãÕó
	BOOL                    mIsAccumBas;          //ÒÔÀÛ¼ÆÇúÏßÎª»ù´¡
    VariableField_Float     mSource[MAXZONE];     //¶àÖÖÔ´»ãÏî
	int                     mSourceDir[MAXZONE];  //Ô´»ãÏîµÄ²¹¸ø·½Ïò
    //ÐèÒª½»»»µÄÊý¾Ý
    CValueArray             mSumFlow;             //¸÷ÖÖÔ´»ãÏîµÄ»ã×Ü²¹¸øÁ÷Á¿
	//ÁÙÊ±Êý¾Ý
	float                   t1_old;
    float                   t2_old;
public:
    Aq_SinkSourceTerm();
    ~Aq_SinkSourceTerm();

//======== ¼Ì³Ðº¯Êý ==========
public:
//ÎªÔËÐÐ×ö×¼±¸
    virtual BOOL Run_Prepare();
//×÷ÓÃµ½º¬Ë®²ã
    virtual BOOL StatusOnAquifer(float t1,float t2);
//¸üÐÂ×´Ì¬
    virtual BOOL StatusChange(float t1,float t2);
//¹Ø±Õ
    virtual BOOL Close();
//======== ÄÚ²¿º¯Êý  ==========
public:
	//¶ÁÐ´ÐÅÏ¢ÎÄ¼þ
	BOOL ReadFromFile();
	BOOL WriteToFile();
	//°´ÕÕÏÖÓÐÐÅÏ¢ÉèÖÃµÚkÖÖÔ´»ãÏî
	BOOL ResetTerm(int kTerm,int dir,CurveLib *pLib,int nNod,float dis,CString cvName);
	//Á´½ÓÇúÏßÊý¾Ý¿âµÄÎÄ¼þ¶ÁÐ´
    BOOL FileExchange_LibLink(CStdioFile *fp, CurveLib *pLib, BOOL isRead=TRUE);
	//½«D3DÕóÁÐÊý¾Ý×ª»»ÎªµãÕóÊý¾Ý
	BOOL TransfD3D_VFF(VariableField_Float *pVF,CIntArray *pId_D3D,CValueArray *pDis_D3D);
};

//µØ±íÕôÉ¢Ä£¿é
class Aq_SurfaceET:public AquiferHydro_Module
{
//========  ÊôÐÔ =============
public:
	CString                 lpFileInf;            //°üº¬±¾Ä£¿éÐÅÏ¢µÄÎÄ¼þ
	CurveLib               *pLibCurve;            //ÇúÏßÊý¾Ý¿âÖ¸Õë
	//Ç±ÔÚÕô·¢Ç¿¶ÈÊÇÒ»¸öÑÝ±äµãÕó
	BOOL                    mIsAccumBas;          //ÒÔÀÛ¼ÆÇúÏßÎª»ù´¡
    VariableField_Float     mVarE0;               //E0 (L/T) µÄ·Ö²¼Ê½ÑÝ±äµãÕó
    CStringArray            mCvDepthET;           //Õô·¢ÏµÊýËæµØÏÂË®ÂñÉî±ä»¯ÇúÏßµÄÃû³ÆÏµÁÐ
	CIntArray               mPointCvET;           //¸÷¸öµØ±íµ¥ÔªÖ¸ÏòÕô·¢ÏµÊýÇúÏßµÄË÷Òý         
    //ÐèÒª½»»»µÄÊý¾Ý
	CValueArray             mHpSurface;           //µØ±íÍÁ²ãµÄÎüÁ¦ÊÆ(µÈÐ§ÎªµØÏÂË®ÂñÉî)
    CValueArray             mETa;                 //·Ö²¼Ê½µØ±íÊµ¼ÊÕôÉ¢Ç¿¶È, È¡¸ºÖµ
public:
    Aq_SurfaceET();
    ~Aq_SurfaceET();
//======== ¼Ì³Ðº¯Êý ==========
public:
//ÎªÔËÐÐ×ö×¼±¸
    virtual BOOL Run_Prepare();
//×÷ÓÃµ½º¬Ë®²ã
    virtual BOOL StatusOnAquifer(float t1,float t2);
//×÷ÓÃµ½Ä£¿é
    virtual BOOL StatusOnModule(float t1,float t2);
//¸üÐÂ×´Ì¬
    virtual BOOL StatusChange(float t1,float t2);
//¹Ø±Õ
    virtual BOOL Close();
//======== ÄÚ²¿º¯Êý  ==========
public:
	BOOL ReadFromFile();
	BOOL WriteToFile();
    //Á´½ÓÇúÏßÊý¾Ý¿âµÄÎÄ¼þ¶ÁÐ´
    BOOL FileExchange_LibLink(CStdioFile *fp, CurveLib *pLib, BOOL isRead=TRUE);
	//ÉèÖÃµ¥Ò»µÄÄ£¿éÊý¾Ý
	BOOL ResetUniform(int nNod, CurveLib *pLib, CString nameCvRe, CString nameCvE0);
	//ÉèÖÃµ¥Ò»Õô·¢¶¯Ì¬ÇúÏßÄ£¿é
    void SetSingleE0(const FCurve &change_E0);
	//ÉèÖÃ¾ßÓÐµ¥Ò»ÏßÐÔÕô·¢ÏµÊýµÄÄ£¿é
	void SetLinear_CvDepth(float depthZeroET, float depthFullET);
	//ÉèÖÃÒ»¸öµ¥ÇúÏßÕô·¢Ä£¿é
    void SetSingle_CvDepth(const FCurve &depthET);
};

//ºÓÁ÷µ¥Ôª²ÎÊý¼¯
struct aPamRiver
{
	int    indexCell;   //º¬Ë®²ãµ¥ÔªµÄÐòºÅ
	int    branch[3];   //»ãÈë±¾µ¥ÔªºÓÁ÷µÄÖ§Á÷ÐòºÅ, ×î¶à3Ìõ
	int    cvDevelop;   //Ë®Àû¹¤³ÌÒýË®Á÷Á¿¶¯Ì¬ÇúÏßË÷Òý
	float  zBot;        //ºÓÁ÷µ×²¿µÄ¸ß¶È
	float  rCqLeak;     //ºÓ´²ÉøÂ©ÏµÊý=ÉøÍ¸ÏµÊý*³¤¶È*¿í¶È/ºñ¶È
    float  bedThick;    //ºÓ´²³Á»ýÎïµÄºñ¶È
    float  powerIndex;  //ÃÝÖ¸Êý(Ë®Éî-Á÷Á¿ÇúÏß)
	//ÔËÐÐÖÐ¼ä±äÁ¿
	float  leakFlow;    //Ö÷¸ÉºÓÁ÷µÄÉøÂ©Á÷Á¿
	float  outFlow;     //Ö÷¸ÉºÓÁ÷»ã³ö±¾µ¥Ôª½øÈëÏÂÓÎµÄÁ÷Á¿
	float  exportFlow;  //Ö÷¸ÉºÓ±»ÈËÀàÒý³öµÄÁ÷Á¿, ÓÃÓÚñîºÏË®Àû¹¤³ÌÄ£ÐÍ
	float  stage;       //Ö÷¸ÉºÓÁ÷µÄË®Î»
	BOOL   isOnTime;    //ÒÑ¾­ÔÚ±¾Ê±¿Ì½øÐÐÁËÅäÖÃ
};
typedef struct aPamRiver RPam;
typedef CArray<RPam,RPam&> RPamArray;  //²ÎÊýÐòÁÐ

//µØ±íºÓÁ÷ÍøÂçÄ£¿é
class Aq_River:public AquiferHydro_Module
{
//========  ÊôÐÔ =============
public:
	CString                 lpFileInf;            //°üº¬±¾Ä£¿éÐÅÏ¢µÄÎÄ¼þ
	CurveLib               *pLibFlow;             //¾¶Á÷Á¿¶¯Ì¬ÇúÏßÊý¾Ý¿âÖ¸Õë

    RPamArray               mRiver[MAXZONE];      //ºÓÁ÷Êý¾Ý½á¹¹
	BOOL                    rActive[MAXZONE];     //ÊÇ·ñ¼¤»îºÓÁ÷
	float                   refDepth[MAXZONE];    //²Î¿¼Ë®Éî
	float                   refQ[MAXZONE];        //²Î¿¼Á÷Á¿
	CString                 refSource[MAXZONE];   //ºÓÁ÷ÆðµãÁ÷Á¿ËæÊ±¼äµÄ±ä»¯ÇúÏß

	CString                 lpFileResult;         //±£´æ¼ÆËã½á¹ûµÄÎÄ¼þ
    CStdioFile              fpResult;             //ÎÄ¼þÖ¸Õë

public:
    Aq_River();
    ~Aq_River();

//======== ¼Ì³Ðº¯Êý ==========
public:
//ÎªÔËÐÐ×ö×¼±¸
    virtual BOOL Run_Prepare();
//×÷ÓÃµ½º¬Ë®²ã
    virtual BOOL StatusOnAquifer(float t1,float t2);
//×÷ÓÃµ½Ä£¿é
    virtual BOOL StatusOnModule(float t1,float t2);
//Ç°½øÒ»²½
	virtual BOOL StepForward(float time=0,BOOL onWriteObs=FALSE,BOOL onWriteOut=FALSE);
//¸üÐÂ×´Ì¬
    virtual BOOL StatusChange(float t1,float t2);
//¹Ø±Õ
    virtual BOOL Close();
//======== ÄÚ²¿º¯Êý  ==========
public:
	//¶ÁÐ´ÐÅÏ¢ÎÄ¼þ
	BOOL ReadFromFile();
	BOOL WriteToFile();
	//ÎÄ¼þ½»»»
    BOOL FileExchange_LibLink(CStdioFile *fp, CurveLib *pLib, BOOL isRead=TRUE);
	//¶ÔÒ»¸öºÓÁ÷µ¥Ôª½øÐÐÅäÖÃ
	BOOL OnTimeCellRiver(int river, int cell, float inFlow);
	BOOL AllRiverCellOnTime();
    void ResetAllRiverCell();
	//³õÊ¼»¯ºÓÁ÷
	BOOL InsetRiverLine(RPam reach,int riv,CPoint locBegin,int rangeCell,int dir);
	//°ÑÒýË®Á÷Á¿ÖØÐÂ¹éÁã(Èç¹ûÓÐÏÖ³ÉµÄË®Àû¹¤³ÌÇúÏß,Ôò·µ»ØFALSE)
    BOOL ResetExportFlow(float t1,float t2);
};

//¾®¿×µ¥Ôª²ÎÊý¼¯
struct aPamWell
{
	int    indexCell;   //º¬Ë®²ãµ¥ÔªµÄÐòºÅ
	BOOL   KnownFlow;   //¾®¿×µÄÐÔÖÊ: KnownFlow=TRUE, ÒÑÖªÁ÷Á¿; KnownFlow=FALSE, ÒÑÖªË®Î».
    float  Rw;          //¾®¿×°ë¾¶
    float  SkinFactor;  //±íÆ¤ÏµÊý
    float  CwLoss;      //¾®ËðÏµÊý, --wxsh 2008.2.2

	float  refQh;       //È±Ê¡Á÷Á¿»òË®Î»
	int    indexQh;     //¾®Á÷Á¿»ò¾®Ë®Î»¶¯Ì¬ÇúÏßµÄË÷ÒýºÅ   
};
typedef struct aPamWell WPam;
typedef CArray<WPam,WPam&> WPamArray;  //²ÎÊýÐòÁÐ

//µ¥²ã¾®¿×Ð£ÕýÄ£¿é
class Aq_Well:public AquiferHydro_Module
{
//========  ÊôÐÔ =============
public:
	CString                 lpFileInf;            //°üº¬±¾Ä£¿éÐÅÏ¢µÄÎÄ¼þ
	CurveLib               *pLibWell;             //¾®Á÷Á¿»ò¾®Ë®Î»¶¯Ì¬ÇúÏßÊý¾Ý¿âÖ¸Õë

    WPamArray               mWells;               //¾®¿×ÏµÁÐ
	CStringArray            mNameQh;              //¾®Á÷Á¿»ò¾®Ë®Î»¶¯Ì¬ÇúÏßÃû³Æ
	
	CValueArray             mCpWell;              //¾®¿×Ð£ÕýÏµÊý: ³ÌÐò×Ô¶¯¼ÆËã
	CValueArray             wFlowRate;            //¾®¿×Á÷Á¿
	CValueArray             wWaterTable;          //¾®¿×Ë®Î»

	CString                 lpFileResult;         //±£´æ¼ÆËã½á¹ûµÄÎÄ¼þ
    CStdioFile              fpResult;             //ÎÄ¼þÖ¸Õë
public:
    Aq_Well();
    ~Aq_Well();

//======== ¼Ì³Ðº¯Êý ==========
public:
//ÎªÔËÐÐ×ö×¼±¸
    virtual BOOL Run_Prepare();
//×÷ÓÃµ½º¬Ë®²ã
    virtual BOOL StatusOnAquifer(float t1,float t2);
//×÷ÓÃµ½Ä£¿é
    virtual BOOL StatusOnModule(float t1,float t2);
//¸üÐÂ×´Ì¬
    virtual BOOL StatusChange(float t1,float t2);
//Ç°½øÒ»²½
	virtual BOOL StepForward(float time=0,BOOL onWriteObs=FALSE,BOOL onWriteOut=FALSE);
//¹Ø±Õ
    virtual BOOL Close();
//======== ÄÚ²¿º¯Êý  ==========
public:
	//¶ÁÐ´ÐÅÏ¢ÎÄ¼þ
	BOOL ReadFromFile();
	BOOL WriteToFile();
	//Á´½ÓÇúÏßÊý¾Ý¿âµÄÎÄ¼þ¶ÁÐ´
    BOOL FileExchange_LibLink(CStdioFile *fp, CurveLib *pLib, BOOL isRead=TRUE);
	//¼ÆËã¾®¿×Ð£ÕýÏµÊý
	BOOL ForWellFactor(BOOL isRepeat=FALSE);
	//Ôö¼ÓÒ»¸ö¾®¿×µ¥Ôª
	BOOL InsertWellCell(WPam wPam,CString QhName=_T(""));
};

//=======================================================================
//=========================  Ä£ÐÍËãÀýº¯Êý  ==============================
//=======================================================================

//ÔËÐÐÄ£ÐÍµÄÏß³Ìº¯Êý
UINT ModelRunOffLine(LPVOID pParam);

//Ê¹ÓÃº¯Êý´´½¨Ä£ÐÍµÄËãÀý
BOOL SampleOnFunction(ModelRunInfo *timeInfo);
//Ê¹ÓÃÎÄ¼þ´´½¨Ä£ÐÍµÄËãÀý
BOOL SampleOnFile(ModelRunInfo *timeInfo);
