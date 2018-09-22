// ******** LK.h ********

#ifndef AMT_LK_H
#define AMT_LK_H


typedef enum EvStatus
{         
	// 	CV_BADMEMBLOCK_ERR          = -113,
	// 	CV_INPLACE_NOT_SUPPORTED_ERR= -112,
	// 	CV_UNMATCHED_ROI_ERR        = -111,
	// 	CV_NOTFOUND_ERR             = -110,
	// 	CV_BADCONVERGENCE_ERR       = -109,

	// 	CV_BADDEPTH_ERR             = -107,
	// 	CV_BADROI_ERR               = -106,
	// 	CV_BADHEADER_ERR            = -105,
	// 	CV_UNMATCHED_FORMATS_ERR    = -104,
	// 	CV_UNSUPPORTED_COI_ERR      = -103,
	// 	CV_UNSUPPORTED_CHANNELS_ERR = -102,
	// 	CV_UNSUPPORTED_DEPTH_ERR    = -101,
	// 	CV_UNSUPPORTED_FORMAT_ERR   = -100,

	// 	CV_BADARG_ERR      = -49,  //ipp comp
	// 	CV_NOTDEFINED_ERR  = -48,  //ipp comp
	// 
	// 	CV_BADCHANNELS_ERR = -47,  //ipp comp
	CV_BADRANGE_ERR    = -44,  //ipp comp
	//	CV_BADSTEP_ERR     = -29,  //ipp comp

	CV_BADFLAG_ERR     =  -12,
	//	CV_DIV_BY_ZERO_ERR =  -11, //ipp comp
	//	CV_BADCOEF_ERR     =  -10,

	//	CV_BADFACTOR_ERR   =  -7,
	//	CV_BADPOINT_ERR    =  -6,
	//	CV_BADSCALE_ERR    =  -4,
	CV_OUTOFMEM_ERR    =  -3,
	CV_NULLPTR_ERR     =  -2,
	CV_BADSIZE_ERR     =  -1,
	CV_OK              =   0
}EvStatus;

#define CV_MAX_THREADS 1

// -256.f ... 511.f
const float iev8x32fTab_cv[] =
{
	-256.f, -255.f, -254.f, -253.f, -252.f, -251.f, -250.f, -249.f,
	-248.f, -247.f, -246.f, -245.f, -244.f, -243.f, -242.f, -241.f,
	-240.f, -239.f, -238.f, -237.f, -236.f, -235.f, -234.f, -233.f,
	-232.f, -231.f, -230.f, -229.f, -228.f, -227.f, -226.f, -225.f,
	-224.f, -223.f, -222.f, -221.f, -220.f, -219.f, -218.f, -217.f,
	-216.f, -215.f, -214.f, -213.f, -212.f, -211.f, -210.f, -209.f,
	-208.f, -207.f, -206.f, -205.f, -204.f, -203.f, -202.f, -201.f,
	-200.f, -199.f, -198.f, -197.f, -196.f, -195.f, -194.f, -193.f,
	-192.f, -191.f, -190.f, -189.f, -188.f, -187.f, -186.f, -185.f,
	-184.f, -183.f, -182.f, -181.f, -180.f, -179.f, -178.f, -177.f,
	-176.f, -175.f, -174.f, -173.f, -172.f, -171.f, -170.f, -169.f,
	-168.f, -167.f, -166.f, -165.f, -164.f, -163.f, -162.f, -161.f,
	-160.f, -159.f, -158.f, -157.f, -156.f, -155.f, -154.f, -153.f,
	-152.f, -151.f, -150.f, -149.f, -148.f, -147.f, -146.f, -145.f,
	-144.f, -143.f, -142.f, -141.f, -140.f, -139.f, -138.f, -137.f,
	-136.f, -135.f, -134.f, -133.f, -132.f, -131.f, -130.f, -129.f,
	-128.f, -127.f, -126.f, -125.f, -124.f, -123.f, -122.f, -121.f,
	-120.f, -119.f, -118.f, -117.f, -116.f, -115.f, -114.f, -113.f,
	-112.f, -111.f, -110.f, -109.f, -108.f, -107.f, -106.f, -105.f,
	-104.f, -103.f, -102.f, -101.f, -100.f,  -99.f,  -98.f,  -97.f,
	-96.f,  -95.f,  -94.f,  -93.f,  -92.f,  -91.f,  -90.f,  -89.f,
	-88.f,  -87.f,  -86.f,  -85.f,  -84.f,  -83.f,  -82.f,  -81.f,
	-80.f,  -79.f,  -78.f,  -77.f,  -76.f,  -75.f,  -74.f,  -73.f,
	-72.f,  -71.f,  -70.f,  -69.f,  -68.f,  -67.f,  -66.f,  -65.f,
	-64.f,  -63.f,  -62.f,  -61.f,  -60.f,  -59.f,  -58.f,  -57.f,
	-56.f,  -55.f,  -54.f,  -53.f,  -52.f,  -51.f,  -50.f,  -49.f,
	-48.f,  -47.f,  -46.f,  -45.f,  -44.f,  -43.f,  -42.f,  -41.f,
	-40.f,  -39.f,  -38.f,  -37.f,  -36.f,  -35.f,  -34.f,  -33.f,
	-32.f,  -31.f,  -30.f,  -29.f,  -28.f,  -27.f,  -26.f,  -25.f,
	-24.f,  -23.f,  -22.f,  -21.f,  -20.f,  -19.f,  -18.f,  -17.f,
	-16.f,  -15.f,  -14.f,  -13.f,  -12.f,  -11.f,  -10.f,   -9.f,
	-8.f,   -7.f,   -6.f,   -5.f,   -4.f,   -3.f,   -2.f,   -1.f,
	0.f,    1.f,    2.f,    3.f,    4.f,    5.f,    6.f,    7.f,
	8.f,    9.f,   10.f,   11.f,   12.f,   13.f,   14.f,   15.f,
	16.f,   17.f,   18.f,   19.f,   20.f,   21.f,   22.f,   23.f,
	24.f,   25.f,   26.f,   27.f,   28.f,   29.f,   30.f,   31.f,
	32.f,   33.f,   34.f,   35.f,   36.f,   37.f,   38.f,   39.f,
	40.f,   41.f,   42.f,   43.f,   44.f,   45.f,   46.f,   47.f,
	48.f,   49.f,   50.f,   51.f,   52.f,   53.f,   54.f,   55.f,
	56.f,   57.f,   58.f,   59.f,   60.f,   61.f,   62.f,   63.f,
	64.f,   65.f,   66.f,   67.f,   68.f,   69.f,   70.f,   71.f,
	72.f,   73.f,   74.f,   75.f,   76.f,   77.f,   78.f,   79.f,
	80.f,   81.f,   82.f,   83.f,   84.f,   85.f,   86.f,   87.f,
	88.f,   89.f,   90.f,   91.f,   92.f,   93.f,   94.f,   95.f,
	96.f,   97.f,   98.f,   99.f,  100.f,  101.f,  102.f,  103.f,
	104.f,  105.f,  106.f,  107.f,  108.f,  109.f,  110.f,  111.f,
	112.f,  113.f,  114.f,  115.f,  116.f,  117.f,  118.f,  119.f,
	120.f,  121.f,  122.f,  123.f,  124.f,  125.f,  126.f,  127.f,
	128.f,  129.f,  130.f,  131.f,  132.f,  133.f,  134.f,  135.f,
	136.f,  137.f,  138.f,  139.f,  140.f,  141.f,  142.f,  143.f,
	144.f,  145.f,  146.f,  147.f,  148.f,  149.f,  150.f,  151.f,
	152.f,  153.f,  154.f,  155.f,  156.f,  157.f,  158.f,  159.f,
	160.f,  161.f,  162.f,  163.f,  164.f,  165.f,  166.f,  167.f,
	168.f,  169.f,  170.f,  171.f,  172.f,  173.f,  174.f,  175.f,
	176.f,  177.f,  178.f,  179.f,  180.f,  181.f,  182.f,  183.f,
	184.f,  185.f,  186.f,  187.f,  188.f,  189.f,  190.f,  191.f,
	192.f,  193.f,  194.f,  195.f,  196.f,  197.f,  198.f,  199.f,
	200.f,  201.f,  202.f,  203.f,  204.f,  205.f,  206.f,  207.f,
	208.f,  209.f,  210.f,  211.f,  212.f,  213.f,  214.f,  215.f,
	216.f,  217.f,  218.f,  219.f,  220.f,  221.f,  222.f,  223.f,
	224.f,  225.f,  226.f,  227.f,  228.f,  229.f,  230.f,  231.f,
	232.f,  233.f,  234.f,  235.f,  236.f,  237.f,  238.f,  239.f,
	240.f,  241.f,  242.f,  243.f,  244.f,  245.f,  246.f,  247.f,
	248.f,  249.f,  250.f,  251.f,  252.f,  253.f,  254.f,  255.f,
	256.f,  257.f,  258.f,  259.f,  260.f,  261.f,  262.f,  263.f,
	264.f,  265.f,  266.f,  267.f,  268.f,  269.f,  270.f,  271.f,
	272.f,  273.f,  274.f,  275.f,  276.f,  277.f,  278.f,  279.f,
	280.f,  281.f,  282.f,  283.f,  284.f,  285.f,  286.f,  287.f,
	288.f,  289.f,  290.f,  291.f,  292.f,  293.f,  294.f,  295.f,
	296.f,  297.f,  298.f,  299.f,  300.f,  301.f,  302.f,  303.f,
	304.f,  305.f,  306.f,  307.f,  308.f,  309.f,  310.f,  311.f,
	312.f,  313.f,  314.f,  315.f,  316.f,  317.f,  318.f,  319.f,
	320.f,  321.f,  322.f,  323.f,  324.f,  325.f,  326.f,  327.f,
	328.f,  329.f,  330.f,  331.f,  332.f,  333.f,  334.f,  335.f,
	336.f,  337.f,  338.f,  339.f,  340.f,  341.f,  342.f,  343.f,
	344.f,  345.f,  346.f,  347.f,  348.f,  349.f,  350.f,  351.f,
	352.f,  353.f,  354.f,  355.f,  356.f,  357.f,  358.f,  359.f,
	360.f,  361.f,  362.f,  363.f,  364.f,  365.f,  366.f,  367.f,
	368.f,  369.f,  370.f,  371.f,  372.f,  373.f,  374.f,  375.f,
	376.f,  377.f,  378.f,  379.f,  380.f,  381.f,  382.f,  383.f,
	384.f,  385.f,  386.f,  387.f,  388.f,  389.f,  390.f,  391.f,
	392.f,  393.f,  394.f,  395.f,  396.f,  397.f,  398.f,  399.f,
	400.f,  401.f,  402.f,  403.f,  404.f,  405.f,  406.f,  407.f,
	408.f,  409.f,  410.f,  411.f,  412.f,  413.f,  414.f,  415.f,
	416.f,  417.f,  418.f,  419.f,  420.f,  421.f,  422.f,  423.f,
	424.f,  425.f,  426.f,  427.f,  428.f,  429.f,  430.f,  431.f,
	432.f,  433.f,  434.f,  435.f,  436.f,  437.f,  438.f,  439.f,
	440.f,  441.f,  442.f,  443.f,  444.f,  445.f,  446.f,  447.f,
	448.f,  449.f,  450.f,  451.f,  452.f,  453.f,  454.f,  455.f,
	456.f,  457.f,  458.f,  459.f,  460.f,  461.f,  462.f,  463.f,
	464.f,  465.f,  466.f,  467.f,  468.f,  469.f,  470.f,  471.f,
	472.f,  473.f,  474.f,  475.f,  476.f,  477.f,  478.f,  479.f,
	480.f,  481.f,  482.f,  483.f,  484.f,  485.f,  486.f,  487.f,
	488.f,  489.f,  490.f,  491.f,  492.f,  493.f,  494.f,  495.f,
	496.f,  497.f,  498.f,  499.f,  500.f,  501.f,  502.f,  503.f,
	504.f,  505.f,  506.f,  507.f,  508.f,  509.f,  510.f,  511.f,
};

#define EV_8TO32F(x)         iev8x32fTab_cv[(x)+256]  

// typedef struct CvPyramid
// {
// 	uchar **ptr;
// 	CvSize *sz;
// 	double *rate;
// 	int *step;
// 	uchar *state;
// 	int level;
// }CvPyramid;


int evCalcOpticalFlowPyrLK( const void* arrA, const void* arrB, void* pyrarrA, void* pyrarrB,
							 const CvPoint2D32f * featuresA, CvPoint2D32f * featuresB,
							 int count, CvSize winSize, int level,
							 char *status, float *error, CvTermCriteria criteria, int flags );

int evGetRectPix(const IplImage* pSrc, IplImage* pDst, CvPoint2D32f center);

EvStatus  ievGetRectSubPix_8u32f_C1R( const uchar* src, int src_step, CvSize src_size,
									  float* dst, int dst_step, CvSize win_size, CvPoint2D32f center );

EvStatus  ievGetRectPix_8u_C1R( const uchar* src, int src_step, CvSize src_size,
							    uchar* dst, int dst_step, CvSize win_size, CvPoint2D32f center );


#endif