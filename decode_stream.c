/*
 *  Copyright (c) 2010-2012, Freescale Semiconductor Inc.,
 *  All Rights Reserved.
 *
 *  The following programs are the sole property of Freescale Semiconductor Inc.,
 *  and contain its proprietary and confidential information.
 *
 */

/*   
 *	decode_stream.c
 *	this file is the interface between unit test and vpu wrapper
 *
 *	History :
 *	Date	(y.m.d)		Author			Version			Description
 *	2010-09-14		eagle zhou		0.1				Created
 */



#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "vpu_wrapper.h"
#include "decode_stream.h"
//#include "fb_render.h"
#include "unistd.h"		//for usleep()

#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/avassert.h"
#include "libavutil/time.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavcodec/avfft.h"
#include "libavcodec/avcodec.h"
#include "libswresample/swresample.h"

#include <linux/videodev2.h>
#include <linux/fb.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include <sys/time.h>

//#include <linux/mxcfb.h>
//#include <linux/mxc_v4l2.h>

#include <ring_buffer.h>

//#define VPU_FILE_MODE_TEST
#define MXCFB_SET_GBL_ALPHA     _IOW('F', 0x21, struct mxcfb_gbl_alpha)
#define DEC_STREAM_PRINTF
#define CHECK_DEAD_LOOP
#ifdef CHECK_DEAD_LOOP
#define MAX_NOTUSED_LOOP	(1000) //(200)
#define MAX_NULL_LOOP		(1000) //(200)
#endif

#define DEC_STREAM_DEBUG_
#ifdef DEC_STREAM_DEBUG
#define DEC_STREAM_PRINTF printf
//#define DEC_TRACE	printf("%s: %d \r\n",__FUNCTION__,__LINE__);
#define DEC_TRACE
#else
#define DEC_TRACE
#endif

#define USLEEP	usleep
#define SLEEP_TIME_US		(1000) //(1000)	

//#define FILL_DATA_UNIT	(16*1024)
#define MAX_FRAME_NUM	(30)
#define FRAME_SURPLUS	(0)//(3)
#define FRAME_ALIGN		(16)

#if 1	//avoid buf is too big to malloc by vpu
#define VPU_DEC_MAX_NUM_MEM_NUM	20
#else
#define VPU_DEC_MAX_NUM_MEM_NUM	VPU_DEC_MAX_NUM_MEM_REQS
#endif

#define Align(ptr,align)	(((unsigned int)ptr+(align)-1)/(align)*(align))

struct mxcfb_gbl_alpha {                                                                                                                       
        int enable;
        int alpha;
};

typedef struct
{
	//virtual mem info
	int nVirtNum;
	unsigned int virtMem[VPU_DEC_MAX_NUM_MEM_NUM];

	//phy mem info
	int nPhyNum;
	unsigned int phyMem_virtAddr[VPU_DEC_MAX_NUM_MEM_NUM];
	unsigned int phyMem_phyAddr[VPU_DEC_MAX_NUM_MEM_NUM];
	unsigned int phyMem_cpuAddr[VPU_DEC_MAX_NUM_MEM_NUM];
	unsigned int phyMem_size[VPU_DEC_MAX_NUM_MEM_NUM];	
}DecMemInfo;


#ifdef VPU_FILE_MODE_TEST
#define FILE_MODE_MAX_FRAME_LEN		(1024*1024)	//1M bytes
#define FILE_MODE_MAX_FRAME_NUM		40
#define FILE_MODE_LOG	printf
unsigned int g_filemode_frame_length[FILE_MODE_MAX_FRAME_NUM]={
	//from zdfhd.ts
	131923, 40129,38102,40147,20749,
	19142,36248,21235,21153,35797,
	33383,21124,19286,38835,18808,
	20024,103980,47883,43615,39538,
	19854,24160,45832,22873,23503,
	41762,36254,24498,20684,35544,
	16691,18975,145114,43136,39354,
	33491,15813,15431,33030,16289,
};
static unsigned int g_filemode_curloc=0;
#endif

#if 1 // timer related part
#include <sys/time.h>
#define TIME_DEC_ID	0
#define TIME_TOTAL_ID	1
#define TIME_RESOLUTION_ID	2
#define MAX_BUFFER_NUM 16

struct testbuffer
{
        unsigned char *start;
        size_t offset;
        unsigned int length;
};

struct testbuffer buffers[MAX_BUFFER_NUM];

int fd_v4l = 0;
int g_frame_size;
int g_num_buffers;

char * g_dev = "/dev/video17";
int g_mem_type = V4L2_MEMORY_MMAP;
int g_overlay = 0;
int g_in_width = 1920;
int g_in_height = 1080;
int g_display_width = 0;
int g_display_height = 0;
int g_display_top = 0;
int g_display_left = 0;
int g_rotate = 0;
int g_vflip = 0;
int g_hflip = 0;
int g_vdi_enable = 0;
int g_vdi_motion = 0;
int g_icrop_w = 0;
int g_icrop_h = 0;
int g_icrop_top = 0;
int g_icrop_left = 0;
uint32_t g_in_fmt = V4L2_PIX_FMT_YUV420;

int g_bmp_header = 0;
int g_loop_count = 1;
int g_frame_period = 33333;

struct v4l2_control ctrl;
struct v4l2_format fmt;
struct v4l2_framebuffer fb;
struct v4l2_cropcap cropcap;
struct v4l2_capability cap;
struct v4l2_fmtdesc fmtdesc;
struct v4l2_crop crop;
struct v4l2_rect icrop;
struct v4l2_buffer buf;

static int frame_cnt = 0;
static int streamon_cnt = 0;
static struct timeval tv_start, tv_current;



static struct timeval time_beg[3];
static struct timeval time_end[3];
static unsigned long long total_time[3];

static void time_init(int id)
{
	total_time[id]=0;
}

static void time_start(int id)
{
	gettimeofday(&time_beg[id], 0);
}

static void time_stop(int id)
{
	unsigned int tm_1, tm_2;
	gettimeofday(&time_end[id], 0);

	tm_1 = time_beg[id].tv_sec * 1000000 + time_beg[id].tv_usec;
	tm_2 = time_end[id].tv_sec * 1000000 + time_end[id].tv_usec;
	total_time[id] = total_time[id] + (tm_2-tm_1);
}

static unsigned long long time_report(int id)
{
	return total_time[id];
}
#endif

int MapTileSpace(int x, int y, int stride, int blockW, int blockH,int field,unsigned char* pTop, unsigned char* pBot, unsigned char** ppTileAddr)
{
	/*	input parameter:
x: x coordinate in linear space
y: y coordinate in linear space
blockW: tile block unit width
blockH: tile block unit height	
output parameter:
pTileOffset: offset in the whole tile buffer
in tile space: every [blockW x blockH] is divided by two sub-block: [blockW/2 x blockH]		
	 */
	int blockNum=0;
	int subblockNum=0;
	int blockLength;
	int subblockLength;
	int subblockOffset;
	int tileOffset=0;
	unsigned char* pBase;

	if(0==field)
	{
		pBase=pTop;
	}
	else
	{		
		if(0==(y&1))
		{
			pBase=pTop;
		}
		else
		{
			pBase=pBot;
		}
		y=y>>1;
	}

	blockLength=blockW*blockH;
	subblockLength=blockLength/2;
	blockNum=(y/blockH)*(stride/blockW)+(x/blockW);
	if((x%blockW)>=(blockW/2))
	{
		subblockNum=1;
	}
	subblockOffset=(y%blockH)*(blockW/2)+(x%(blockW/2));
	tileOffset=blockNum*blockLength+subblockNum*subblockLength+subblockOffset;
	//printf("field: %d, stride: %d, block[%dx%d], linear: [x,y]: [%d,%d],  tile offset: %d \r\n",field,stride,blockW,blockH,x,y,tileOffset);
	*ppTileAddr=pBase+tileOffset;

	return 1;	
}

int ConvertDataFromTileToLinear(int maptype, int width, int height, 
		unsigned char* pSrcYTop,unsigned char* pSrcYBot,unsigned char* pSrcCbTop,unsigned char* pSrcCbBot,
		unsigned char* pDstY,unsigned char* pDstCb,unsigned char* pDstCr)
{
	int x,y,j;
	unsigned char* pTileBlockAddr;
	unsigned char temp_buf[8];
	int blockW,blockH;
	int field=0;

	//fill Y buffer
	if(maptype==1)
	{
		//frame tile: luma--w(8+8)xh16
		blockW=16;
		blockH=16;
		field=0;
	}
	else
	{
		//field tile: luma--w(8+8)xh8
		blockW=16;
		blockH=8;
		field=1;
	}
	for(y=0;y<height;y++)
	{
		for(x=0;x<width;x+=8)
		{
			MapTileSpace(x, y, width, blockW, blockH,field,pSrcYTop,pSrcYBot, &pTileBlockAddr);
			memcpy(pDstY+y*width+x, pTileBlockAddr, 8);
		}
	}

	//fill Cb/Cr buffer
	if(maptype==1)
	{
		//frame tile: Chroma--w(8+8)xh8
		blockW=16;
		blockH=8;
		field=0;
	}
	else
	{
		//field tile: Chroma--w(8+8)xh4
		blockW=16;
		blockH=4;
		field=1;
	}
	for(y=0;y<height/2;y++)
	{
		for(x=0;x<width;x+=8)
		{
			MapTileSpace(x, y, width, blockW, blockH,field,pSrcCbTop,pSrcCbBot,&pTileBlockAddr);
			memcpy(temp_buf, pTileBlockAddr, 8);
			for (j = 0; j < 4; j++)
			{
				*pDstCb++=*(temp_buf+j*2);
				*pDstCr++=*(temp_buf+j*2+1);
			}
		}
	}

	return 1;
}

int ConvertCodecFormat(int codec, VpuCodStd* pCodec)
{
	switch (codec)
	{
		case 1:
			*pCodec=VPU_V_MPEG2;
			break;
		case 2:
			*pCodec=VPU_V_MPEG4;
			break;			
		case 3:
			*pCodec=VPU_V_DIVX3;
			break;
		case 4:
			*pCodec=VPU_V_DIVX4;
			break;
		case 5:
			*pCodec=VPU_V_DIVX56;
			break;
		case 6:
			*pCodec=VPU_V_XVID;
			break;			
		case 7:
			*pCodec=VPU_V_H263;
			break;
		case 8:
			*pCodec=VPU_V_AVC;
			break;
		case 9:
			*pCodec=VPU_V_VC1; //VPU_V_VC1_AP
			break;
		case 10:
			*pCodec=VPU_V_RV;
			break;			
		case 11:
			*pCodec=VPU_V_MJPG;
			break;
		case 12:
			*pCodec=VPU_V_AVS;
			break;
		case 13:
			*pCodec=VPU_V_VP8;
			break;
		case 14:
			*pCodec=VPU_V_AVC_MVC;
			break;
		default:
			return 0;			
	}
	return 1;
}


int ConvertSkipMode(int skip, VpuDecConfig* pConfig,int * pPara)
{
	switch (skip)
	{
		case 0:
			*pConfig=VPU_DEC_CONF_SKIPMODE;
			*pPara=VPU_DEC_SKIPNONE;
			DEC_STREAM_PRINTF("normal mode \r\n");
			break;
		case 1:
			*pConfig=VPU_DEC_CONF_SKIPMODE;
			*pPara=VPU_DEC_SKIPPB;
			DEC_STREAM_PRINTF("skip PB frames \r\n");
			break;
		case 2:
			*pConfig=VPU_DEC_CONF_SKIPMODE;
			*pPara=VPU_DEC_SKIPB;
			DEC_STREAM_PRINTF("skip B frames \r\n");
			break;			
		case 3:
			*pConfig=VPU_DEC_CONF_SKIPMODE;
			*pPara=VPU_DEC_SKIPALL;
			DEC_STREAM_PRINTF("skip all frames \r\n");
			break;
		case 4:
			*pConfig=VPU_DEC_CONF_SKIPMODE;
			*pPara=VPU_DEC_ISEARCH;	//only search I, not skip I
			DEC_STREAM_PRINTF("I frame search \r\n");
			break;
		default:
			DEC_STREAM_PRINTF("unsupported skip mode: %d \r\n",skip);
			return 0;
	}
	return 1;
}

int FreeMemBlockFrame(DecMemInfo* pDecMem, int nFrmNum)
{
	VpuMemDesc vpuMem;
	VpuDecRetCode vpuRet;	
	int cnt=0;
	int retOk=1;
	int i;

	//free physical mem
	for(i=pDecMem->nPhyNum-1;i>=0;i--)
	{
		vpuMem.nPhyAddr=pDecMem->phyMem_phyAddr[i];
		vpuMem.nVirtAddr=pDecMem->phyMem_virtAddr[i];
		vpuMem.nCpuAddr=pDecMem->phyMem_cpuAddr[i];
		vpuMem.nSize=pDecMem->phyMem_size[i];
		vpuRet=VPU_DecFreeMem(&vpuMem);
		if(vpuRet!=VPU_DEC_RET_SUCCESS)
		{
			DEC_STREAM_PRINTF("%s: free vpu memory failure : ret=%d \r\n",__FUNCTION__,vpuRet);
			retOk=0;
		}
		cnt++;
		if(cnt==nFrmNum) break;
	}
	pDecMem->nPhyNum=pDecMem->nPhyNum-cnt;
	if(cnt!=nFrmNum) 
	{
		DEC_STREAM_PRINTF("error: only freed %d frames, required frame numbers: %d \r\n",cnt,nFrmNum);
		retOk=0;
	}	
	return retOk;
}

int FreeMemBlock(DecMemInfo* pDecMem)
{
	int i;
	VpuMemDesc vpuMem;
	VpuDecRetCode vpuRet;
	int retOk=1;

	//free virtual mem
	for(i=0;i<pDecMem->nVirtNum;i++)
	{
		if((void*)pDecMem->virtMem[i]) free((void*)pDecMem->virtMem[i]);
	}
	pDecMem->nVirtNum=0;

	//free physical mem
	for(i=0;i<pDecMem->nPhyNum;i++)
	{
		vpuMem.nPhyAddr=pDecMem->phyMem_phyAddr[i];
		vpuMem.nVirtAddr=pDecMem->phyMem_virtAddr[i];
		vpuMem.nCpuAddr=pDecMem->phyMem_cpuAddr[i];
		vpuMem.nSize=pDecMem->phyMem_size[i];
		vpuRet=VPU_DecFreeMem(&vpuMem);
		if(vpuRet!=VPU_DEC_RET_SUCCESS)
		{
			DEC_STREAM_PRINTF("%s: free vpu memory failure : ret=%d \r\n",__FUNCTION__,vpuRet);
			retOk=0;
		}
	}
	pDecMem->nPhyNum	=0;

	return retOk;
}


int MallocMemBlock(VpuMemInfo* pMemBlock,DecMemInfo* pDecMem)
{
	int i;
	unsigned char * ptr=NULL;
	int size;

	for(i=0;i<pMemBlock->nSubBlockNum;i++)
	{
		size=pMemBlock->MemSubBlock[i].nAlignment+pMemBlock->MemSubBlock[i].nSize;
		if(pMemBlock->MemSubBlock[i].MemType==VPU_MEM_VIRT)
		{
			ptr=(unsigned char *)malloc(size);
			if(ptr==NULL)
			{
				DEC_STREAM_PRINTF("%s: get virtual memory failure, size=%d \r\n",__FUNCTION__,size);
				goto failure;
			}		
			pMemBlock->MemSubBlock[i].pVirtAddr=(unsigned char*)Align(ptr,pMemBlock->MemSubBlock[i].nAlignment);

			//record virtual base addr
			pDecMem->virtMem[pDecMem->nVirtNum]=(unsigned int)ptr;
			pDecMem->nVirtNum++;
		}
		else// if(memInfo.MemSubBlock[i].MemType==VPU_MEM_PHY)
		{
			VpuMemDesc vpuMem;
			VpuDecRetCode ret;
			vpuMem.nSize=size;
			ret=VPU_DecGetMem(&vpuMem);
			if(ret!=VPU_DEC_RET_SUCCESS)
			{
				DEC_STREAM_PRINTF("%s: get vpu memory failure, size=%d, ret=%d \r\n",__FUNCTION__,size,ret);
				goto failure;
			}		
			pMemBlock->MemSubBlock[i].pVirtAddr=(unsigned char*)Align(vpuMem.nVirtAddr,pMemBlock->MemSubBlock[i].nAlignment);
			pMemBlock->MemSubBlock[i].pPhyAddr=(unsigned char*)Align(vpuMem.nPhyAddr,pMemBlock->MemSubBlock[i].nAlignment);

			//record physical base addr
			pDecMem->phyMem_phyAddr[pDecMem->nPhyNum]=(unsigned int)vpuMem.nPhyAddr;
			pDecMem->phyMem_virtAddr[pDecMem->nPhyNum]=(unsigned int)vpuMem.nVirtAddr;
			pDecMem->phyMem_cpuAddr[pDecMem->nPhyNum]=(unsigned int)vpuMem.nCpuAddr;
			pDecMem->phyMem_size[pDecMem->nPhyNum]=size;
			pDecMem->nPhyNum++;			
		}
	}	

	return 1;

failure:
	FreeMemBlock(pDecMem);
	return 0;

}

int ResetBitstream(DecContxt * pDecContxt,int offset)
{
	fseek(pDecContxt->fin,offset,SEEK_SET);
	return 1;
}


int ReadBitstream(DecContxt * pDecContxt, unsigned char* pBitstream,int length)
{
	int readbytes;
	//static int totalReadSize=0;

	//DEC_STREAM_PRINTF("read %d bytes \r\n",length);
	readbytes=fread(pBitstream,1,length,pDecContxt->fin);

	//totalReadSize+=readbytes;
	//printf("total read size: %d \r\n",totalReadSize);
	return readbytes;
}

void fb_setup(void)
{
        struct mxcfb_gbl_alpha alpha;
	int fd;

#ifdef BUILD_FOR_ANDROID
	fd = open("/dev/graphics/fb0",O_RDWR);
#else
	fd = open("/dev/fb0",O_RDWR);
#endif

        alpha.alpha = 0;
        alpha.enable = 1;
        if (ioctl(fd, MXCFB_SET_GBL_ALPHA, &alpha) < 0) {
		printf("set alpha %d failed for fb0\n",  alpha.alpha);
        }

	close(fd);
}

int mxc_v4l_output_test(VpuFrameBuffer* pFrame, int ySize, int uvSize)
{
        int err = 0;
        int retval = 0;
	int type;

	if(frame_cnt == 0){
		gettimeofday(&tv_start, 0);
		printf("start time = %d s, %d us\n", tv_start.tv_sec, tv_start.tv_usec);
	}

	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	buf.memory = g_mem_type;
	if (frame_cnt < g_num_buffers) {
		buf.index = frame_cnt;
		if (ioctl(fd_v4l, VIDIOC_QUERYBUF, &buf) < 0)
		{
			printf("VIDIOC_QUERYBUF failed\n");
			retval = -1;
			return retval;
		}
		if (g_mem_type == V4L2_MEMORY_USERPTR) {
			buf.m.userptr = (unsigned long) buffers[frame_cnt].offset;
			buf.length = buffers[frame_cnt].length;
		}
	}
	else {
		buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		buf.memory = g_mem_type;
		if (ioctl(fd_v4l, VIDIOC_DQBUF, &buf) < 0)
		{
			printf("VIDIOC_DQBUF failed\n");
			retval = -1;
			return retval;
		}
	}
	// skip over bmp header in each frame

	memcpy(buffers[buf.index].start, pFrame->pbufVirtY, ySize);
	memcpy(buffers[buf.index].start + ySize, pFrame->pbufVirtCb, uvSize);
	memcpy(buffers[buf.index].start + ySize +uvSize, pFrame->pbufVirtCr, uvSize);
	
	buf.timestamp.tv_sec = tv_start.tv_sec;
	buf.timestamp.tv_usec = tv_start.tv_usec + (g_frame_period * frame_cnt);

	if ((retval = ioctl(fd_v4l, VIDIOC_QBUF, &buf)) < 0)
	{
		printf("VIDIOC_QBUF failed %d\n", retval);
		retval = -1;
		return retval;
	}

	if ( frame_cnt == streamon_cnt ) { // Start playback after buffers queued
		type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		if (ioctl (fd_v4l, VIDIOC_STREAMON, &type) < 0) {
			printf("Could not start stream\n");
			retval = -1;
			return retval;
		}
		/*simply set fb0 alpha to 0*/
		fb_setup();
	}

	frame_cnt ++;
	
	return 0;
}

int mxc_v4l_output_setup(struct v4l2_format *fmt)
{
	int i, retval;
        struct v4l2_requestbuffers buf_req;

        if (ioctl(fd_v4l, VIDIOC_S_FMT, fmt) < 0)
        {
                printf("set format failed\n");
                return 1;
        }

        if (ioctl(fd_v4l, VIDIOC_G_FMT, fmt) < 0)
        {
                printf("get format failed\n");
                return 1;
        }

        memset(&buf_req, 0, sizeof(buf_req));
        buf_req.count = 4;
        buf_req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf_req.memory = g_mem_type;
        if (ioctl(fd_v4l, VIDIOC_REQBUFS, &buf_req) < 0)
        {
                printf("request buffers failed\n");
                return 1;
        }
        g_num_buffers = buf_req.count;
        printf("v4l2_output test: Allocated %d buffers\n", buf_req.count);

	memset(&buf, 0, sizeof(buf));

	if (g_mem_type == V4L2_MEMORY_MMAP) {
		for (i = 0; i < g_num_buffers; i++) {
			memset(&buf, 0, sizeof (buf));
			buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			buf.memory = g_mem_type;
			buf.index = i;
			if (ioctl(fd_v4l, VIDIOC_QUERYBUF, &buf) < 0)
			{
				printf("VIDIOC_QUERYBUF error\n");
				retval = -1;
				for (i = 0; i < g_num_buffers; i++) {
					munmap (buffers[i].start, buffers[i].length);
				}

			}

			buffers[i].length = buf.length;
			buffers[i].offset = (size_t) buf.m.offset;
			printf("VIDIOC_QUERYBUF: length = %d, offset = %d\n",
				buffers[i].length, buffers[i].offset);
			buffers[i].start = mmap (NULL, buffers[i].length,
					PROT_READ | PROT_WRITE, MAP_SHARED,
					fd_v4l, buffers[i].offset);
			if (buffers[i].start == NULL) {
				printf("v4l2_out test: mmap failed\n");
				retval = -1;
				for (i = 0; i < g_num_buffers; i++) {
					munmap (buffers[i].start, buffers[i].length);
				}
			}
		}
	}


        return 0;
}


int OutputFrame(DecContxt * pDecContxt,VpuDecOutFrameInfo* pOutFrame,int width, int height,int fbHandle,int frameNum)
{
	int ySize;
	int uvSize;
	int retval;
	VpuFrameBuffer* pFrame=pOutFrame->pDisplayFrameBuf;
	//VpuCodStd codecFormat;
	VpuFrameBuffer sLinearFrame;
	VpuMemDesc vpuMem;
	int NeedConvert=0;
	int wrlen;

	/*DEC_STREAM_PRINTF("dynamic resolution: [width x height]=[%d x %d]: crop: [left, top, right, bottom]=[%d, %d, %d, %d] \r\n",
	  pOutFrame->pExtInfo->nFrmWidth,pOutFrame->pExtInfo->nFrmHeight,
	  pOutFrame->pExtInfo->FrmCropRect.nLeft,pOutFrame->pExtInfo->FrmCropRect.nTop,
	  pOutFrame->pExtInfo->FrmCropRect.nRight,pOutFrame->pExtInfo->FrmCropRect.nBottom);*/
	//DEC_STREAM_PRINTF("output one frame, [width x height]=[%d x %d] \r\n",width,height);
	//DEC_STREAM_PRINTF("output one frame, y:0x%X, u:0x%X, v:0x%X \r\n",(unsigned int)pFrame->pbufVirtY,(unsigned int)pFrame->pbufVirtCb,(unsigned int)pFrame->pbufVirtCr);
	ySize=Align(width, FRAME_ALIGN)*Align(height,FRAME_ALIGN);
	switch(pDecContxt->eOutColorFmt)
	{
		case DEC_OUT_420:
			uvSize=ySize/4;
			break;
		case DEC_OUT_422H:
		case DEC_OUT_422V:
			uvSize=ySize/2;
			break;			
		case DEC_OUT_444:
			uvSize=ySize;
			break;
		case DEC_OUT_400:
			uvSize=0;
			break;
		default:
			uvSize=ySize/4;
			break;
	}

	if((pDecContxt->nMapType!=0) && (pDecContxt->nTile2LinearEnable==0)
			&& (pDecContxt->fout || fbHandle))
	{
		NeedConvert=1;
	}

	if(NeedConvert)
	{
		VpuDecRetCode ret;
		memset(&vpuMem,0,sizeof(VpuMemDesc));
		memset(&sLinearFrame,0,sizeof(VpuFrameBuffer));
		vpuMem.nSize=ySize+uvSize+uvSize;
		ret=VPU_DecGetMem(&vpuMem);
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu malloc tile buf failure: ret=%d, size: %d \r\n",__FUNCTION__,ret,vpuMem.nSize);
			goto FAIL;
		}

#if 1
		if(pDecContxt->fout)
		{
			//for debug: output nv12 tile file
			FILE* fp;
			fp = fopen("temp_tile.tile", "ab");
			if(pDecContxt->nMapType==2)		// tile field
			{
				wrlen=fwrite(pFrame->pbufVirtY,1,ySize/2,fp);
				wrlen=fwrite(pFrame->pbufVirtY_tilebot,1,ySize/2,fp);
				wrlen=fwrite(pFrame->pbufVirtCb,1,uvSize,fp);
				wrlen=fwrite(pFrame->pbufVirtCb_tilebot,1,uvSize,fp);
			}
			else		// tile frame
			{
				wrlen=fwrite(pFrame->pbufVirtY,1,ySize,fp);
				wrlen=fwrite(pFrame->pbufVirtCb,1,2*uvSize,fp);
				//fwrite(pFrame->pbufVirtCr,1,uvSize,fp);				
			}
			fclose(fp);
		}	
#endif
		//FIXME: now only care pbufVirtY/Cb/Cr three address
		sLinearFrame.pbufVirtY=(unsigned char*)vpuMem.nVirtAddr;
		sLinearFrame.pbufVirtCb=sLinearFrame.pbufVirtY+ySize;
		sLinearFrame.pbufVirtCr=sLinearFrame.pbufVirtCb+uvSize;
		//printf("convert tile space: YTob: 0x%X, YBot: 0x%X, CbTop: 0x%X, CbBot: 0x%X \r\n",pFrame->pbufVirtY, pFrame->pbufVirtY_tilebot, pFrame->pbufVirtCb, pFrame->pbufVirtCb_tilebot);
		ConvertDataFromTileToLinear(pDecContxt->nMapType, width, height, 
				pFrame->pbufVirtY, pFrame->pbufVirtY_tilebot, pFrame->pbufVirtCb, pFrame->pbufVirtCb_tilebot, 
				sLinearFrame.pbufVirtY,sLinearFrame.pbufVirtCb, sLinearFrame.pbufVirtCr);
		pFrame=&sLinearFrame;
	}

	DEC_TRACE;
	//output file
	if(pDecContxt->fout)
	{
		wrlen=fwrite(pFrame->pbufVirtY,1,ySize,pDecContxt->fout);
		wrlen=fwrite(pFrame->pbufVirtCb,1,uvSize,pDecContxt->fout);
		wrlen=fwrite(pFrame->pbufVirtCr,1,uvSize,pDecContxt->fout);
	}
	DEC_TRACE;
	//display 
	if(pDecContxt->nDisplay)
	{
        retval = mxc_v4l_output_test(pFrame, ySize,uvSize);
		//fb_render_drawYUVframe(fbHandle, pFrame->pbufVirtY, pFrame->pbufVirtCb, pFrame->pbufVirtCr, Align(width, FRAME_ALIGN), Align(height,FRAME_ALIGN));
	}

	//ConvertCodecFormat(pDecContxt->nCodec, &codecFormat);
	switch(pOutFrame->ePicType)
	{
		case VPU_I_PIC:
			DEC_STREAM_PRINTF("frame : %d (I) \r\n",frameNum);
			break;
		case VPU_P_PIC:
			DEC_STREAM_PRINTF("frame : %d (P) \r\n",frameNum);
			break;
		case VPU_B_PIC:
			DEC_STREAM_PRINTF("frame : %d (B) \r\n",frameNum);
			break;
		case VPU_BI_PIC:
			DEC_STREAM_PRINTF("frame : %d (BI) \r\n",frameNum);
			break;			
		case VPU_IDR_PIC:
			DEC_STREAM_PRINTF("frame : %d (IDR) \r\n",frameNum);
			break;			
		case VPU_SKIP_PIC:
			DEC_STREAM_PRINTF("frame : %d (SKIP) \r\n",frameNum);
			break;			
		default:
			DEC_STREAM_PRINTF("frame : %d (*) \r\n",frameNum);
			break;
	}

	if(NeedConvert)
	{
		VPU_DecFreeMem(&vpuMem);	
	}
	return 1;	
FAIL:
	return 0;
}



int RenderInit()
{
	
	printf("render init\n");
	int retval;

	if ((fd_v4l = open(g_dev, O_RDWR, 0)) < 0)
	{
		printf("Unable to open %s\n", g_dev);
		retval = 1;
		goto err0;
	}

	if (!ioctl(fd_v4l, VIDIOC_QUERYCAP, &cap)) {
		printf("driver=%s, card=%s, bus=%s, "
				"version=0x%08x, "
				"capabilities=0x%08x\n",
				cap.driver, cap.card, cap.bus_info,
				cap.version,
				cap.capabilities);
	}

	fmtdesc.index = 0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	while (!ioctl(fd_v4l, VIDIOC_ENUM_FMT, &fmtdesc)) {
		printf("fmt %s: fourcc = 0x%08x\n",
				fmtdesc.description,
				fmtdesc.pixelformat);
		fmtdesc.index++;
	}

        memset(&cropcap, 0, sizeof(cropcap));
        cropcap.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (ioctl(fd_v4l, VIDIOC_CROPCAP, &cropcap) < 0)
        {
                printf("get crop capability failed\n");
                retval = 1;
                goto err1;
        }
        printf("cropcap.bounds.width = %d\ncropcap.bound.height = %d\n" \
               "cropcap.defrect.width = %d\ncropcap.defrect.height = %d\n",
               cropcap.bounds.width, cropcap.bounds.height,
               cropcap.defrect.width, cropcap.defrect.height);

        crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        crop.c.top = g_display_top;
        crop.c.left = g_display_left;
        crop.c.width = g_display_width;
        crop.c.height = g_display_height;
        if (ioctl(fd_v4l, VIDIOC_S_CROP, &crop) < 0)
        {
                printf("set crop failed\n");
                retval = 1;
                goto err1;
        }

        // Set rotation
        ctrl.id = V4L2_CID_ROTATE;
        ctrl.value = g_rotate;
        if (ioctl(fd_v4l, VIDIOC_S_CTRL, &ctrl) < 0)
        {
                printf("set ctrl rotate failed\n");
                retval = 1;
                goto err1;
        }
        ctrl.id = V4L2_CID_VFLIP;
        ctrl.value = g_vflip;
        if (ioctl(fd_v4l, VIDIOC_S_CTRL, &ctrl) < 0)
        {
                printf("set ctrl vflip failed\n");
                retval = 1;
                goto err1;
        }
        ctrl.id = V4L2_CID_HFLIP;
        ctrl.value = g_hflip;
        if (ioctl(fd_v4l, VIDIOC_S_CTRL, &ctrl) < 0)
        {
                printf("set ctrl hflip failed\n");
                retval = 1;
                goto err1;
        }


	fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
	fb.flags = V4L2_FBUF_FLAG_PRIMARY;
        ioctl(fd_v4l, VIDIOC_S_FBUF, &fb);
	 memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmt.fmt.pix.width=g_in_width;
        fmt.fmt.pix.height=g_in_height;
        fmt.fmt.pix.pixelformat = g_in_fmt;

	retval = mxc_v4l_output_setup(&fmt);
	if (retval < 0)
		goto err1;

	g_frame_size = fmt.fmt.pix.sizeimage;
	
	return 0;


err1:
        close(fd_v4l);
err0:
        return retval;
}

int ProcessInitInfo(DecContxt * pDecContxt,VpuDecHandle handle,VpuDecInitInfo* pInitInfo,DecMemInfo* pDecMemInfo, int*pOutFrmNum)
{
	VpuDecRetCode ret;
	VpuFrameBuffer frameBuf[MAX_FRAME_NUM];
	VpuMemDesc vpuMem;
	int BufNum;
	int i;
	int totalSize=0;
	int mvSize=0;
	int ySize=0;
	int uSize=0;
	int vSize=0;
	int yStride=0;
	int uStride=0;
	int vStride=0;
	unsigned char* ptr;
	unsigned char* ptrVirt;
	int nAlign;
	int multifactor=1;

	//get init info	
	DEC_TRACE;
	ret=VPU_DecGetInitialInfo(handle, pInitInfo);
	DEC_TRACE;
	if(VPU_DEC_RET_SUCCESS!=ret)
	{
		DEC_STREAM_PRINTF("%s: vpu get init info failure: ret=%d \r\n",__FUNCTION__,ret);	
		return 0;
	}
	//malloc frame buffs
	BufNum=pInitInfo->nMinFrameBufferCount+FRAME_SURPLUS;
	if(BufNum>MAX_FRAME_NUM)
	{
		DEC_STREAM_PRINTF("%s: vpu request too many frames : num=0x%X \r\n",__FUNCTION__,pInitInfo->nMinFrameBufferCount);	
		return 0;		
	}

	yStride=Align(pInitInfo->nPicWidth,FRAME_ALIGN);
	if(pInitInfo->nInterlace)
	{
		ySize=Align(pInitInfo->nPicWidth,FRAME_ALIGN)*Align(pInitInfo->nPicHeight,(2*FRAME_ALIGN));
	}
	else
	{
		ySize=Align(pInitInfo->nPicWidth,FRAME_ALIGN)*Align(pInitInfo->nPicHeight,FRAME_ALIGN);
	}

	//for MJPG: we need to check 4:4:4/4:2:2/4:2:0/4:0:0
	{
		VpuCodStd vpuCodec=0;
		ConvertCodecFormat(pDecContxt->nCodec, &vpuCodec);
		if(VPU_V_MJPG==vpuCodec)
		{
			switch(pInitInfo->nMjpgSourceFormat)
			{
				case 0:	//4:2:0
					DEC_STREAM_PRINTF("MJPG: 4:2:0 \r\n");
					uStride=yStride/2;
					vStride=uStride;
					uSize=ySize/4;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_420;
					break;
				case 1:	//4:2:2 hor
					DEC_STREAM_PRINTF("MJPG: 4:2:2 hor \r\n");
					uStride=yStride/2;
					vStride=uStride;
					uSize=ySize/2;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_422H;
					break;
				case 2:	//4:2:2 ver
					DEC_STREAM_PRINTF("MJPG: 4:2:2 ver \r\n");				
					uStride=yStride;
					vStride=uStride;
					uSize=ySize/2;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_422V;
					break;
				case 3:	//4:4:4
					DEC_STREAM_PRINTF("MJPG: 4:4:4 \r\n");				
					uStride=yStride;
					vStride=uStride;
					uSize=ySize;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_444;
					break;
				case 4:	//4:0:0
					DEC_STREAM_PRINTF("MJPG: 4:0:0 \r\n");				
					uStride=0;
					vStride=uStride;
					uSize=0;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_400;
					break;
				default:	//4:2:0
					DEC_STREAM_PRINTF("unknown color format: %d \r\n",vpuCodec);
					uStride=yStride/2;
					vStride=uStride;
					uSize=ySize/4;
					vSize=uSize;	
					mvSize=uSize;
					pDecContxt->eOutColorFmt=DEC_OUT_420;
					break;			
			}
		}
		else
		{
			//4:2:0 for all video
			uStride=yStride/2;
			vStride=uStride;
			uSize=ySize/4;
			vSize=uSize;	
			mvSize=uSize;
			pDecContxt->eOutColorFmt=DEC_OUT_420;
		}
	}

	nAlign=pInitInfo->nAddressAlignment;
	if(pDecContxt->nMapType==2)
	{
		//only consider Y since interleave must be enabled
		multifactor=2;	//for field, we need to consider alignment for top and bot
	}
	if(nAlign>1)
	{
		ySize=Align(ySize,multifactor*nAlign);
		uSize=Align(uSize,nAlign);
		vSize=Align(vSize,nAlign);
	}

#if 1	// avoid buffer is too big to malloc by vpu
	for(i=0;i<BufNum;i++)
	{
		totalSize=(ySize+uSize+vSize+mvSize+nAlign)*1;

		vpuMem.nSize=totalSize;
		DEC_TRACE;
		ret=VPU_DecGetMem(&vpuMem);
		DEC_TRACE;
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu malloc frame buf failure: ret=%d \r\n",__FUNCTION__,ret);	
			return 0;
		}
		//record memory info for release
		pDecMemInfo->phyMem_phyAddr[pDecMemInfo->nPhyNum]=vpuMem.nPhyAddr;
		pDecMemInfo->phyMem_virtAddr[pDecMemInfo->nPhyNum]=vpuMem.nVirtAddr;
		pDecMemInfo->phyMem_cpuAddr[pDecMemInfo->nPhyNum]=vpuMem.nCpuAddr;
		pDecMemInfo->phyMem_size[pDecMemInfo->nPhyNum]=vpuMem.nSize;
		pDecMemInfo->nPhyNum++;

		//fill frameBuf
		ptr=(unsigned char*)vpuMem.nPhyAddr;
		ptrVirt=(unsigned char*)vpuMem.nVirtAddr;

		/*align the base address*/
		if(nAlign>1)
		{
			ptr=(unsigned char*)Align(ptr,nAlign);
			ptrVirt=(unsigned char*)Align(ptrVirt,nAlign);
		}

		/* fill stride info */
		frameBuf[i].nStrideY=yStride;
		frameBuf[i].nStrideC=uStride;	

		/* fill phy addr*/
		frameBuf[i].pbufY=ptr;
		frameBuf[i].pbufCb=ptr+ySize;
		frameBuf[i].pbufCr=ptr+ySize+uSize;
		frameBuf[i].pbufMvCol=ptr+ySize+uSize+vSize;
		//ptr+=ySize+uSize+vSize+mvSize;
		/* fill virt addr */
		frameBuf[i].pbufVirtY=ptrVirt;
		frameBuf[i].pbufVirtCb=ptrVirt+ySize;
		frameBuf[i].pbufVirtCr=ptrVirt+ySize+uSize;
		frameBuf[i].pbufVirtMvCol=ptrVirt+ySize+uSize+vSize;
		//ptrVirt+=ySize+uSize+vSize+mvSize;

		/* fill bottom address for field tile*/
		if(pDecContxt->nMapType==2)
		{
			frameBuf[i].pbufY_tilebot=frameBuf[i].pbufY+ySize/2;
			frameBuf[i].pbufCb_tilebot=frameBuf[i].pbufCr;
			frameBuf[i].pbufVirtY_tilebot=frameBuf[i].pbufVirtY+ySize/2;
			frameBuf[i].pbufVirtCb_tilebot=frameBuf[i].pbufVirtCr;			
		}
		else
		{
			frameBuf[i].pbufY_tilebot=0;
			frameBuf[i].pbufCb_tilebot=0;
			frameBuf[i].pbufVirtY_tilebot=0;
			frameBuf[i].pbufVirtCb_tilebot=0;
		}
	}
#else
	mvSize=Align(mvSize,nAlign);
	totalSize=(ySize+uSize+vSize+mvSize)*BufNum+nAlign;

	vpuMem.nSize=totalSize;
	DEC_TRACE;
	ret=VPU_DecGetMem(&vpuMem);
	DEC_TRACE;
	if(VPU_DEC_RET_SUCCESS!=ret)
	{
		DEC_STREAM_PRINTF("%s: vpu malloc frame buf failure: ret=%d \r\n",__FUNCTION__,ret);	
		return 0;
	}
	//record memory info for release
	pDecMemInfo->phyMem_phyAddr[pDecMemInfo->nPhyNum]=vpuMem.nPhyAddr;
	pDecMemInfo->phyMem_virtAddr[pDecMemInfo->nPhyNum]=vpuMem.nVirtAddr;
	pDecMemInfo->phyMem_cpuAddr[pDecMemInfo->nPhyNum]=vpuMem.nCpuAddr;
	pDecMemInfo->phyMem_size[pDecMemInfo->nPhyNum]=vpuMem.nSize;
	pDecMemInfo->nPhyNum++;

	//fill frameBuf
	ptr=(unsigned char*)vpuMem.nPhyAddr;
	ptrVirt=(unsigned char*)vpuMem.nVirtAddr;

	/*align the base address*/
	if(nAlign>1)
	{
		ptr=(unsigned char*)Align(ptr,nAlign);
		ptrVirt=(unsigned char*)Align(ptrVirt,nAlign);
	}

	for(i=0;i<BufNum;i++)
	{
		/* fill stride info */
		frameBuf[i].nStrideY=yStride;
		frameBuf[i].nStrideC=uStride;	

		/* fill phy addr*/
		frameBuf[i].pbufY=ptr;
		frameBuf[i].pbufCb=ptr+ySize;
		frameBuf[i].pbufCr=ptr+ySize+uSize;
		frameBuf[i].pbufMvCol=ptr+ySize+uSize+vSize;
		ptr+=ySize+uSize+vSize+mvSize;
		/* fill virt addr */
		frameBuf[i].pbufVirtY=ptrVirt;
		frameBuf[i].pbufVirtCb=ptrVirt+ySize;
		frameBuf[i].pbufVirtCr=ptrVirt+ySize+uSize;
		frameBuf[i].pbufVirtMvCol=ptrVirt+ySize+uSize+vSize;
		ptrVirt+=ySize+uSize+vSize+mvSize;

		/* fill bottom address for field tile*/
		if(pDecContxt->nMapType==2)
		{
			frameBuf[i].pbufY_tilebot=frameBuf[i].pbufY+ySize/2;
			frameBuf[i].pbufCb_tilebot=frameBuf[i].pbufCr;
			frameBuf[i].pbufVirtY_tilebot=frameBuf[i].pbufVirtY+ySize/2;
			frameBuf[i].pbufVirtCb_tilebot=frameBuf[i].pbufVirtCr;			
		}
		else
		{
			frameBuf[i].pbufY_tilebot=0;
			frameBuf[i].pbufCb_tilebot=0;
			frameBuf[i].pbufVirtY_tilebot=0;
			frameBuf[i].pbufVirtCb_tilebot=0;
		}		
	}
#endif

	//register frame buffs
	DEC_TRACE;
	ret=VPU_DecRegisterFrameBuffer(handle, frameBuf, BufNum);
	DEC_TRACE;
	if(VPU_DEC_RET_SUCCESS!=ret)
	{
		DEC_STREAM_PRINTF("%s: vpu register frame failure: ret=%d \r\n",__FUNCTION__,ret);	
		return 0;
	}	

	*pOutFrmNum=BufNum;
	return 1;
}


int DecodeLoop(VpuDecHandle handle,DecContxt * pDecContxt, unsigned char* pBitstream,DecMemInfo* pDecMemInfo,int* pFbHandle,AVFormatContext *pFormatCtx, AVPacket *packet, char* sps_pps, int sps_pps_length, int videoindex)
{
	int err;
	int i;
	int fileeos;
	int dispeos;
	int readbytes=0;
	int bufNull;
	int dispFrameNum;
	int repeatNum=pDecContxt->nRepeatNum;
	int unitDataValidNum;
	int unitDataSize=pDecContxt->nUnitDataSize;
	VpuDecRetCode ret;
	VpuBufferNode InData;
	int bufRetCode=0;
	VpuDecInitInfo InitInfo;
	VpuDecOutFrameInfo frameInfo;
	unsigned long long totalTime=0;
	int capability=0;
	VpuDecFrameLengthInfo decFrmLengthInfo;
	unsigned int totalDecConsumedBytes;	//stuffer + frame
	int nFrmNum;
	int firstFrame = 1;
	int ferr;
	rbuffer *frm_buffer;

	frm_buffer = (rbuffer *)malloc(sizeof(rbuffer));
	ferr = init_ring_buffer(frm_buffer, 1024 * 2048);

#ifdef CHECK_DEAD_LOOP
	int NotUsedLoopCnt=0;
	int NULLDataLoopCnt=0;	
#endif

	VPU_DecGetCapability(handle, VPU_DEC_CAP_FRAMESIZE, &capability);
	DEC_STREAM_PRINTF("capability: report frame size supported: %d \r\n",capability);

RepeatPlay:

	//reset init value
	err=0;
	fileeos=0;
	dispeos=0;	
	dispFrameNum=0;
	unitDataValidNum=0;
	totalDecConsumedBytes=0;

	time_init(TIME_DEC_ID);
	time_init(TIME_TOTAL_ID);
	time_start(TIME_TOTAL_ID);

#ifdef VPU_FILE_MODE_TEST
	g_filemode_curloc=0;
#endif

	//init buff status
	bufNull=1;

	//here, we use the one config for the whole stream
	{
		VpuDecConfig config;		
		int param;

		//config skip type
		if(0==ConvertSkipMode(pDecContxt->nSkipMode,&config,&param))
		{
			DEC_STREAM_PRINTF("unvalid skip mode: %d, ignored \r\n",pDecContxt->nSkipMode);
			config=VPU_DEC_CONF_SKIPMODE;
			param=VPU_DEC_SKIPNONE;
		}
		ret=VPU_DecConfig(handle, config, &param);
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu config failure: config=0x%X, ret=%d \r\n",__FUNCTION__,(unsigned int)config,ret);
			err=1;
			goto Exit;
		}	

		//config delay buffer size
		if(pDecContxt->nDelayBufSize>=0)
		{
			config=VPU_DEC_CONF_BUFDELAY;
			param=pDecContxt->nDelayBufSize;
			DEC_STREAM_PRINTF("set delay buffer size: %d bytes \r\n",param);
			ret=VPU_DecConfig(handle, config, &param);
			if(VPU_DEC_RET_SUCCESS!=ret)
			{
				DEC_STREAM_PRINTF("%s: vpu config failure: config=0x%X, ret=%d \r\n",__FUNCTION__,(unsigned int)config,ret);
				err=1;
				goto Exit;
			}
		}

		//config input type: normal
		config=VPU_DEC_CONF_INPUTTYPE;
		param=VPU_DEC_IN_NORMAL;
		DEC_STREAM_PRINTF("set input type : normal(%d)  \r\n",param);
		ret=VPU_DecConfig(handle, config, &param);
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu config failure: config=0x%X, ret=%d \r\n",__FUNCTION__,(unsigned int)config,ret);
			err=1;
			goto Exit;
		}
	}

	//main loop for playing
	while((av_read_frame(pFormatCtx, packet)>=0) && (packet->stream_index==videoindex))  //read and parse one frame
	{
		printf("read one frame, size:%d\n", packet->size);
		packet->data[0] = 0;	
		packet->data[1] = 0;	
		packet->data[2] = 0;	
		packet->data[3] = 1;	

		if(firstFrame){   //if first frame,need to add sps and pps data
			firstFrame = 0;
			frm_buffer = input_data(frm_buffer, sps_pps, sps_pps_length);
			frm_buffer = input_data(frm_buffer, packet->data, packet->size + 4 - 4);//mp4 will format the last four data of one frame to 0 0 0 0,so cut the last four data
			
		}
		else{
			frm_buffer = input_data(frm_buffer, packet->data, packet->size + 4 - 4);//mp4 will format the last four data of one frame to 0 0 0 0,so cut the last four data
		}

		while(frm_buffer->data_num >= unitDataSize){	
			printf("data:%d\n", frm_buffer->data_num);
			if(bufNull){
				//read new data into bitstream buf
				frm_buffer = output_data(frm_buffer, pBitstream, unitDataSize);

				if(unitDataSize!=readbytes)
				{
					//EOS
					DEC_STREAM_PRINTF("%s: read file end: last size=0x%X \r\n",__FUNCTION__,readbytes);
					fileeos=1;
				}
				bufNull=0;
			}
			else
			{
				//skip read
				//DEC_STREAM_PRINTF("skip read  \r\n");
			}
			//DEC_STREAM_PRINTF("readbytes: %d , bufRetCode: %d  \r\n",readbytes,bufRetCode);
			//decode bitstream buf
			InData.nSize=unitDataSize;
			InData.pPhyAddr=NULL;
			InData.pVirAddr=pBitstream;
			InData.sCodecData.pData=NULL;
			InData.sCodecData.nSize=0;
			if((pDecContxt->nCodec==13)||((pDecContxt->nCodec==3)))
			{
				//backdoor:  to notify vpu this is raw data. vpu wrapper shouldn't add other additional headers.
#ifndef IMX5_PLATFORM  //1 check iMX5 or iMX6 ??
				InData.sCodecData.nSize=0xFFFFFFFF;	//for iMX6: VP8, DivX3
#else			
				InData.sCodecData.nSize=0;	//for iMX5:DivX3:  (1)file mode;  (2) stream mode(need to feed .avi ?)
#endif
			}
			DEC_TRACE;
			time_init(TIME_RESOLUTION_ID);
			time_start(TIME_RESOLUTION_ID);		
			time_start(TIME_DEC_ID);
			ret=VPU_DecDecodeBuf(handle, &InData,&bufRetCode);
			printf("output status:%x\n", bufRetCode);
			time_stop(TIME_DEC_ID);
			DEC_TRACE;
			//DEC_STREAM_PRINTF("%s: bufRetCode=0x%X \r\n",__FUNCTION__,bufRetCode);	
			if(VPU_DEC_RET_SUCCESS!=ret)
			{
				DEC_STREAM_PRINTF("%s: vpu dec buf failure: ret=%d \r\n",__FUNCTION__,ret);	
				err=1;
				break;
			}

			//check input buff	
			if(bufRetCode&VPU_DEC_INPUT_USED)
			{
				bufNull=1;
				unitDataValidNum++;
			}

			//check init info
			if(bufRetCode&VPU_DEC_INIT_OK)
			{
				//process init info
				if(0==ProcessInitInfo(pDecContxt,handle,&InitInfo,pDecMemInfo,&nFrmNum))
				{
					DEC_STREAM_PRINTF("%s: vpu process init info failure: \r\n",__FUNCTION__);
					err=1;
					break;
				}
				printf("%s: Init OK, [width x height]=[%d x %d], Interlaced: %d, MinFrm: %d \r\n",__FUNCTION__,InitInfo.nPicWidth,InitInfo.nPicHeight,InitInfo.nInterlace,InitInfo.nMinFrameBufferCount);

				if(pDecContxt->nDisplay)
				{
					RenderInit();
				}
			}

			//check resolution change
			if(bufRetCode&VPU_DEC_RESOLUTION_CHANGED)
			{
				long long ts;
				DEC_STREAM_PRINTF("receive resolution changed event, will release previous frames %d \r\n",nFrmNum);
				//time_init(TIME_RESOLUTION_ID);
				//time_start(TIME_RESOLUTION_ID);
				//release previous frames
				FreeMemBlockFrame(pDecMemInfo, nFrmNum);
				//get new init info
				if(0==ProcessInitInfo(pDecContxt,handle,&InitInfo,pDecMemInfo,&nFrmNum))
				{
					DEC_STREAM_PRINTF("%s: vpu process re-init info failure: \r\n",__FUNCTION__);
					err=1;
					break;
				}
				time_stop(TIME_RESOLUTION_ID);
				ts=time_report(TIME_RESOLUTION_ID);
				DEC_STREAM_PRINTF("%s: Re-Init OK, [width x height]=[%d x %d], Interlaced: %d, MinFrm: %d \r\n",__FUNCTION__,InitInfo.nPicWidth,InitInfo.nPicHeight,InitInfo.nInterlace,InitInfo.nMinFrameBufferCount);
				DEC_STREAM_PRINTF("time for process of resolution change event is %lld(us) \r\n",ts);
				if(pDecContxt->nDisplay)
				{
					RenderInit();
				}
				if(pDecContxt->fout)
				{
					DEC_STREAM_PRINTF("seek to head of write file \r\n");
					fseek(pDecContxt->fout,0,SEEK_SET);
				}
			}

			//check frame size
			if(capability)
			{
				if(bufRetCode&VPU_DEC_ONE_FRM_CONSUMED)
				{
					ret=VPU_DecGetConsumedFrameInfo(handle, &decFrmLengthInfo);
					if(VPU_DEC_RET_SUCCESS!=ret)
					{
						DEC_STREAM_PRINTF("%s: vpu get consumed frame info failure: ret=%d \r\n",__FUNCTION__,ret);	
						err=1;
						break;
					}
					totalDecConsumedBytes+=decFrmLengthInfo.nFrameLength+decFrmLengthInfo.nStuffLength;
					DEC_STREAM_PRINTF("[total:0x%X]:one frame is consumed: 0x%X, consumed total size: %d (stuff size: %d, frame size: %d) \r\n",totalDecConsumedBytes,(unsigned int)decFrmLengthInfo.pFrame,decFrmLengthInfo.nStuffLength+decFrmLengthInfo.nFrameLength,decFrmLengthInfo.nStuffLength,decFrmLengthInfo.nFrameLength);
				}
			}

			//check output buff
			if((bufRetCode&VPU_DEC_OUTPUT_DIS)||(bufRetCode&VPU_DEC_OUTPUT_MOSAIC_DIS))
			{
				//get output frame
				ret=VPU_DecGetOutputFrame(handle, &frameInfo);
				if(VPU_DEC_RET_SUCCESS!=ret)
				{
					DEC_STREAM_PRINTF("%s: vpu get output frame failure: ret=%d \r\n",__FUNCTION__,ret);	
					err=1;
					break;
				}			

				//update display frame count
				dispFrameNum++;

				//display or output frame buff
				DEC_TRACE;
				OutputFrame(pDecContxt,&frameInfo,InitInfo.nPicWidth,InitInfo.nPicHeight,*pFbHandle,dispFrameNum);
				DEC_TRACE;

				//clear frame display flag
				ret=VPU_DecOutFrameDisplayed(handle,frameInfo.pDisplayFrameBuf);
				if(VPU_DEC_RET_SUCCESS!=ret)
				{
					DEC_STREAM_PRINTF("%s: vpu clear frame display failure: ret=%d \r\n",__FUNCTION__,ret);	
					err=1;
					break;
				}
				//DEC_STREAM_PRINTF("return frame: 0x%X \r\n",(unsigned int)frameInfo.pDisplayFrameBuf);
			}
			else if (bufRetCode&VPU_DEC_OUTPUT_EOS)
			{
				dispeos=1;
			}
			else if (bufRetCode&VPU_DEC_OUTPUT_NODIS)
			{
				DEC_TRACE;
			}
			else if (bufRetCode&VPU_DEC_OUTPUT_REPEAT)
			{
				DEC_TRACE;
			}
			else if (bufRetCode&VPU_DEC_OUTPUT_DROPPED)
			{
				DEC_TRACE;
			}
			else
			{
				DEC_TRACE;
			}

			//check whether some frames are skipped by vpu
			if(bufRetCode&VPU_DEC_SKIP)
			{
				//need to get one time stamp to sync the count between frame and timestamp
			}

			//other cases ?
			if(0==(bufRetCode&VPU_DEC_OUTPUT_DIS))
			{
				USLEEP(SLEEP_TIME_US);
			}
		}
	
	}

	if(frm_buffer->data_num){	//last run, to finish decoding
		frm_buffer = output_data(frm_buffer, pBitstream, frm_buffer->data_num);
		InData.nSize=frm_buffer->data_num;
		InData.pPhyAddr=NULL;
		InData.pVirAddr=pBitstream;
		InData.sCodecData.pData=NULL;
		InData.sCodecData.nSize=0;
		if((pDecContxt->nCodec==13)||((pDecContxt->nCodec==3)))
		{
			//backdoor:  to notify vpu this is raw data. vpu wrapper shouldn't add other additional headers.
#ifndef IMX5_PLATFORM  //1 check iMX5 or iMX6 ??
			InData.sCodecData.nSize=0xFFFFFFFF;	//for iMX6: VP8, DivX3
#else			
			InData.sCodecData.nSize=0;	//for iMX5:DivX3:  (1)file mode;  (2) stream mode(need to feed .avi ?)
#endif
		}
		DEC_TRACE;
		time_init(TIME_RESOLUTION_ID);
		time_start(TIME_RESOLUTION_ID);		
		time_start(TIME_DEC_ID);
		ret=VPU_DecDecodeBuf(handle, &InData,&bufRetCode);
		printf("output status:%x\n", bufRetCode);
		time_stop(TIME_DEC_ID);
		DEC_TRACE;
		//DEC_STREAM_PRINTF("%s: bufRetCode=0x%X \r\n",__FUNCTION__,bufRetCode);	
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu dec buf failure: ret=%d \r\n",__FUNCTION__,ret);	
			err=1;
		}

		//check input buff	
		if(bufRetCode&VPU_DEC_INPUT_USED)
		{
			bufNull=1;
			unitDataValidNum++;
		}

		//check init info
		if(bufRetCode&VPU_DEC_INIT_OK)
		{
			//process init info
			if(0==ProcessInitInfo(pDecContxt,handle,&InitInfo,pDecMemInfo,&nFrmNum))
			{
				DEC_STREAM_PRINTF("%s: vpu process init info failure: \r\n",__FUNCTION__);
				err=1;
			}
			printf("%s: Init OK, [width x height]=[%d x %d], Interlaced: %d, MinFrm: %d \r\n",__FUNCTION__,InitInfo.nPicWidth,InitInfo.nPicHeight,InitInfo.nInterlace,InitInfo.nMinFrameBufferCount);

			if(pDecContxt->nDisplay)
			{
				RenderInit();
			}
		}

		//check resolution change
		if(bufRetCode&VPU_DEC_RESOLUTION_CHANGED)
		{
			long long ts;
			DEC_STREAM_PRINTF("receive resolution changed event, will release previous frames %d \r\n",nFrmNum);
			//time_init(TIME_RESOLUTION_ID);
			//time_start(TIME_RESOLUTION_ID);
			//release previous frames
			FreeMemBlockFrame(pDecMemInfo, nFrmNum);
			//get new init info
			if(0==ProcessInitInfo(pDecContxt,handle,&InitInfo,pDecMemInfo,&nFrmNum))
			{
				DEC_STREAM_PRINTF("%s: vpu process re-init info failure: \r\n",__FUNCTION__);
				err=1;
			}
			time_stop(TIME_RESOLUTION_ID);
			ts=time_report(TIME_RESOLUTION_ID);
			DEC_STREAM_PRINTF("%s: Re-Init OK, [width x height]=[%d x %d], Interlaced: %d, MinFrm: %d \r\n",__FUNCTION__,InitInfo.nPicWidth,InitInfo.nPicHeight,InitInfo.nInterlace,InitInfo.nMinFrameBufferCount);
			DEC_STREAM_PRINTF("time for process of resolution change event is %lld(us) \r\n",ts);
			if(pDecContxt->nDisplay)
			{
				RenderInit();
			}
			if(pDecContxt->fout)
			{
				DEC_STREAM_PRINTF("seek to head of write file \r\n");
				fseek(pDecContxt->fout,0,SEEK_SET);
			}
		}

		//check frame size
		if(capability)
		{
			if(bufRetCode&VPU_DEC_ONE_FRM_CONSUMED)
			{
				ret=VPU_DecGetConsumedFrameInfo(handle, &decFrmLengthInfo);
				if(VPU_DEC_RET_SUCCESS!=ret)
				{
					DEC_STREAM_PRINTF("%s: vpu get consumed frame info failure: ret=%d \r\n",__FUNCTION__,ret);	
					err=1;
				}
				totalDecConsumedBytes+=decFrmLengthInfo.nFrameLength+decFrmLengthInfo.nStuffLength;
				DEC_STREAM_PRINTF("[total:0x%X]:one frame is consumed: 0x%X, consumed total size: %d (stuff size: %d, frame size: %d) \r\n",totalDecConsumedBytes,(unsigned int)decFrmLengthInfo.pFrame,decFrmLengthInfo.nStuffLength+decFrmLengthInfo.nFrameLength,decFrmLengthInfo.nStuffLength,decFrmLengthInfo.nFrameLength);
			}
		}

		//check output buff
		if((bufRetCode&VPU_DEC_OUTPUT_DIS)||(bufRetCode&VPU_DEC_OUTPUT_MOSAIC_DIS))
		{
			//get output frame
			ret=VPU_DecGetOutputFrame(handle, &frameInfo);
			if(VPU_DEC_RET_SUCCESS!=ret)
			{
				DEC_STREAM_PRINTF("%s: vpu get output frame failure: ret=%d \r\n",__FUNCTION__,ret);	
				err=1;
			}			

			//update display frame count
			dispFrameNum++;

			//display or output frame buff
			DEC_TRACE;
			OutputFrame(pDecContxt,&frameInfo,InitInfo.nPicWidth,InitInfo.nPicHeight,*pFbHandle,dispFrameNum);
			DEC_TRACE;

			//clear frame display flag
			ret=VPU_DecOutFrameDisplayed(handle,frameInfo.pDisplayFrameBuf);
			if(VPU_DEC_RET_SUCCESS!=ret)
			{
				DEC_STREAM_PRINTF("%s: vpu clear frame display failure: ret=%d \r\n",__FUNCTION__,ret);	
				err=1;
			}
			//DEC_STREAM_PRINTF("return frame: 0x%X \r\n",(unsigned int)frameInfo.pDisplayFrameBuf);
		}
		else if (bufRetCode&VPU_DEC_OUTPUT_EOS)
		{
			dispeos=1;
		}
		else if (bufRetCode&VPU_DEC_OUTPUT_NODIS)
		{
			DEC_TRACE;
		}
		else if (bufRetCode&VPU_DEC_OUTPUT_REPEAT)
		{
			DEC_TRACE;
		}
		else if (bufRetCode&VPU_DEC_OUTPUT_DROPPED)
		{
			DEC_TRACE;
		}
		else
		{
			DEC_TRACE;
		}

		//check whether some frames are skipped by vpu
		if(bufRetCode&VPU_DEC_SKIP)
		{
			//need to get one time stamp to sync the count between frame and timestamp
		}

		//other cases ?
		if(0==(bufRetCode&VPU_DEC_OUTPUT_DIS))
		{
			USLEEP(SLEEP_TIME_US);
		}

	}

#if 1
	//config input type: drain
	VpuDecConfig config;		
	int param;
	config=VPU_DEC_CONF_INPUTTYPE;
	param=VPU_DEC_IN_DRAIN;
	//DEC_STREAM_PRINTF("set input type : drain(%d)  \r\n",param);
	ret=VPU_DecConfig(handle, config, &param);
	if(VPU_DEC_RET_SUCCESS!=ret)
	{
		DEC_STREAM_PRINTF("%s: vpu config failure: config=0x%X, ret=%d \r\n",__FUNCTION__,(unsigned int)config,ret);
		err=1;
	}


	//check other options when no err
	if(err==0)
	{
		//flush bits and frames, it is useful when 'nMaxNum' option is used
		//we should call VPU_DecFlushAll() before close vpu, since some operations may be missing(after enabled nUintDataNum), such as vpu_DecGetOutputInfo() !!!
		ret=VPU_DecFlushAll(handle);
		if(VPU_DEC_RET_SUCCESS!=ret)
		{
			DEC_STREAM_PRINTF("%s: vpu flush failure: ret=%d \r\n",__FUNCTION__,ret);
			err=1;
			goto Exit;
		}

		if(repeatNum>0)
		{
			repeatNum--;
			//replay stream, for VC1/VP8/RV/DIVX3, we need to seek to frame boundary !!!
			DEC_STREAM_PRINTF("repeat: seek to offset: %d(0x%X) bytes \r\n",pDecContxt->nOffset,pDecContxt->nOffset);
			ResetBitstream(pDecContxt,pDecContxt->nOffset);
			goto RepeatPlay;
		}
	}
#endif
Exit:
	if(pDecContxt->nDisplay)
        	close(fd_v4l);
	int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ioctl (fd_v4l, VIDIOC_STREAMOFF, &type);

cleanup:
	if (g_mem_type == V4L2_MEMORY_MMAP) {
		for (i = 0; i < g_num_buffers; i++) {
			munmap (buffers[i].start, buffers[i].length);
		}
	}

	time_stop(TIME_TOTAL_ID);
	//set output info for user
	pDecContxt->nFrameNum=dispFrameNum;
	pDecContxt->nWidth=InitInfo.nPicWidth;
	pDecContxt->nHeight=InitInfo.nPicHeight;
	pDecContxt->nErr=err;
	totalTime=time_report(TIME_DEC_ID);
	pDecContxt->nDecFps=(unsigned long long)1000000*dispFrameNum/totalTime;
	totalTime=time_report(TIME_TOTAL_ID);
	pDecContxt->nTotalFps=(unsigned long long)1000000*dispFrameNum/totalTime;	

	return ((err==0)?1:0);

}

int find_sps_pps(char *filename, char *sps_pps)   //find the sps and pps infomation of the given mp4 file                    
{
	FILE *fp;
	char * fileHead;
	int readNum;
	int i;
	int num_of_spspps;
	fileHead = (char *)malloc(1024 * sizeof(char));
	if((fp = fopen(filename, "r")) == NULL){
		printf("file open error\n");
		free(fileHead);
		return 1;
	}
	else{
		if((readNum = fread(fileHead, 1, 1024, fp)) != 1024){
			printf("file read error %d\n", readNum);
			fclose(fp);
			free(fileHead);
			return 1;	
		}
		else{
			for(i = 0; i < 1024; i++){
				if((fileHead[i] == 0x61) && (fileHead[i+1] == 0x76) &&(fileHead[i+2] == 0x63) &&(fileHead[i+3] == 0x43)){
					sps_pps[0] = 0;
					sps_pps[1] = 0;
					sps_pps[2] = 0;
					sps_pps[3] = 1;
					memcpy(sps_pps+4, fileHead+i+12, fileHead[i+11]); //sps
					sps_pps[4+fileHead[i+11]+0] = 0;
					sps_pps[4+fileHead[i+11]+1] = 0;
					sps_pps[4+fileHead[i+11]+2] = 0;
					sps_pps[4+fileHead[i+11]+3] = 1;
					memcpy(sps_pps+fileHead[i+11]+8, fileHead+i+11+fileHead[i+11]+4, fileHead[i+11+fileHead[i+11]+3]);//pps
					num_of_spspps = fileHead[i+11] + fileHead[i+11+fileHead[i+11]+3] + 8;
					fclose(fp);
					if(fileHead)
						free(fileHead);
					return num_of_spspps;
				}
			}			
		}
	}

}

int decode_stream(DecContxt * pDecContxt, char *infile)
{
	DecMemInfo decMemInfo;
	VpuDecRetCode ret;
	VpuVersionInfo ver;
	VpuWrapperVersionInfo w_ver;
	VpuMemInfo memInfo;
	VpuDecHandle handle;
	VpuDecOpenParam decOpenParam;
	int fbHandle=(int)NULL;
	int noerr;
	int i;
	int capability=0;

	AVFormatContext *pFormatCtx; 
	int              videoindex;  
	AVCodecContext  *pCodecCtx;  
	AVCodec         *pCodec;  
	char * sps_pps;
	int sps_pps_length;

	if(pDecContxt->fout == NULL){
		pDecContxt->nDisplay = 1;
	}
	else
		pDecContxt->nDisplay = 0;

	unsigned char* pBitstream=NULL;
#ifdef VPU_FILE_MODE_TEST		
	int nUnitDataSize=FILE_MODE_MAX_FRAME_LEN;
#else
	int nUnitDataSize=pDecContxt->nUnitDataSize;
#endif
	if(pDecContxt->nCodec == 8)
	{
		printf("codec is H264\n");
		sps_pps = (char *)malloc(30 * sizeof(char)); 
		printf("spspps\n");
		sps_pps_length = find_sps_pps(infile, sps_pps);
		printf("ret is %d\n", sps_pps_length);
		for(i = 0; i < sps_pps_length; i++){
			printf("%x\n", sps_pps[i]);
		}
	}

	//FFmpeg init
	av_register_all();  
	avformat_network_init();  
	pFormatCtx = avformat_alloc_context();  
	printf("1\n");
	if(avformat_open_input(&pFormatCtx,infile,NULL,NULL)!=0){  //open the file and get foamat information
		printf("无法打开文件\n");  
		return -1;  
	}  
	printf("2\n");
	if(avformat_find_stream_info(pFormatCtx, NULL)<0) 
	{  
		printf("Couldn't find stream information.\n");  
		return -1;  
	}  
	videoindex=-1;  
	for(i=0; i<pFormatCtx->nb_streams; i++)   //find the video/audio information
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO)  
		{  
			videoindex=i;  
			break;  
		}  
	if(videoindex==-1)  
	{  
		printf("Didn't find a video stream.\n");  
		return -1;  
	}  
	pCodecCtx=pFormatCtx->streams[videoindex]->codec;  //find the codec according to the stream

	printf("video information-----------------------------------------\n");  
	av_dump_format(pFormatCtx,0,infile,0);  //dump the information
	printf("-------------------------------------------------\n");  



	//alloc bitstream buffer
	pBitstream=malloc(nUnitDataSize);
	if(NULL==pBitstream)
	{
		DEC_STREAM_PRINTF("%s: alloc bitstream buf failure: size=0x%X \r\n",__FUNCTION__,nUnitDataSize);
		return 0;
	}

	//clear 0
	memset(&memInfo,0,sizeof(VpuMemInfo));
	memset(&decMemInfo,0,sizeof(DecMemInfo));

	//load vpu
	ret=VPU_DecLoad();
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu load failure: ret=%d \r\n",__FUNCTION__,ret);
		//goto finish;		
		free(pBitstream);
		return 0;
	}

	//version info
	ret=VPU_DecGetVersionInfo(&ver);
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu get version failure: ret=%d \r\n",__FUNCTION__,ret);
		goto finish;
	}
	DEC_STREAM_PRINTF("vpu lib version : major.minor.rel=%d.%d.%d \r\n",ver.nLibMajor,ver.nLibMinor,ver.nLibRelease);
	DEC_STREAM_PRINTF("vpu fw version : major.minor.rel_rcode=%d.%d.%d_r%d \r\n",ver.nFwMajor,ver.nFwMinor,ver.nFwRelease,ver.nFwCode);

	//wrapper version info
	ret=VPU_DecGetWrapperVersionInfo(&w_ver);
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu get wrapper version failure: ret=%d \r\n",__FUNCTION__,ret);
		goto finish;
	}
	DEC_STREAM_PRINTF("vpu wrapper version : major.minor.rel=%d.%d.%d: %s \r\n",w_ver.nMajor,w_ver.nMinor,w_ver.nRelease,w_ver.pBinary);

	//query memory
	ret=VPU_DecQueryMem(&memInfo);
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu query memory failure: ret=%d \r\n",__FUNCTION__,ret);
		goto finish;		
	}

	//malloc memory for vpu wrapper
	if(0==MallocMemBlock(&memInfo,&decMemInfo))
	{
		DEC_STREAM_PRINTF("%s: malloc memory failure: \r\n",__FUNCTION__);
		goto finish;		
	}

	//set open params
	if(0==ConvertCodecFormat(pDecContxt->nCodec, &decOpenParam.CodecFormat))
	{
		DEC_STREAM_PRINTF("%s: unsupported codec format: id=%d \r\n",__FUNCTION__,pDecContxt->nCodec);
		goto finish;		
	}	

	decOpenParam.nReorderEnable=1;	//for H264
#ifdef VPU_FILE_MODE_TEST	
	decOpenParam.nEnableFileMode=1;	//unit test: using file mode
#else
	decOpenParam.nEnableFileMode=0;	//unit test: using stream mode
#endif	
	//if(decOpenParam.CodecFormat==VPU_V_DIVX3)
	//{
	//	decOpenParam.nPicWidth=pDecContxt->nInWidth;
	//	decOpenParam.nPicHeight=pDecContxt->nInHeight;
	//}

	//check capabilities
	VPU_DecGetCapability((VpuDecHandle)NULL, VPU_DEC_CAP_FILEMODE, &capability);
	DEC_STREAM_PRINTF("capability: file mode supported: %d \r\n",capability);
	VPU_DecGetCapability((VpuDecHandle)NULL, VPU_DEC_CAP_TILE, &capability);
	DEC_STREAM_PRINTF("capability: tile format supported: %d \r\n",capability);
	if((capability==0)&&(pDecContxt->nMapType!=0))
	{
		DEC_STREAM_PRINTF("ERROR: tile format is not supported \r\n");
	}

	decOpenParam.nChromaInterleave=pDecContxt->nChromaInterleave;
	decOpenParam.nMapType=pDecContxt->nMapType;
	decOpenParam.nTiled2LinearEnable=pDecContxt->nTile2LinearEnable;
	//open vpu
	ret=VPU_DecOpen(&handle, &decOpenParam, &memInfo);
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu open failure: ret=%d \r\n",__FUNCTION__,ret);
		goto finish;
	}	

	int y_size = pCodecCtx->width * pCodecCtx->height;  	
	AVPacket *packet=(AVPacket *)malloc(sizeof(AVPacket));  //allocate memory for AVPacket
	av_new_packet(packet, y_size);  


	//decoding loop
	noerr=DecodeLoop(handle, pDecContxt, pBitstream,&decMemInfo,&fbHandle, pFormatCtx, packet, sps_pps, sps_pps_length, videoindex);

	if(0==noerr)
	{
		DEC_STREAM_PRINTF("%s: vpu reset: handle=0x%X \r\n",__FUNCTION__,handle);
		VPU_DecReset(handle);
	}
	//else
	{
		//close vpu
		ret=VPU_DecClose(handle);
		if (ret!=VPU_DEC_RET_SUCCESS)
		{
			DEC_STREAM_PRINTF("%s: vpu close failure: ret=%d \r\n",__FUNCTION__,ret);
		}	

	}

finish:
	//release mem
	if(0==FreeMemBlock(&decMemInfo))
	{
		DEC_STREAM_PRINTF("%s: free memory failure:  \r\n",__FUNCTION__);
	}

	if(pBitstream)
	{
		free(pBitstream);
	}

	//unload
	ret=VPU_DecUnLoad();
	if (ret!=VPU_DEC_RET_SUCCESS)
	{
		DEC_STREAM_PRINTF("%s: vpu unload failure: ret=%d \r\n",__FUNCTION__,ret);
	}
	return 1;
}

int decode_reset()
{
	DEC_STREAM_PRINTF("reset decoder (all instances) \r\n");
	VPU_DecReset(0);
	return 1;
}


