#include <stdint.h>
#include <stddef.h>

enum TypTag
{
  OTTypVer = 1,
  OTTypEdg,
  OTTypTri,
  OTTypQad,
  OTTypHex,
  OTNmbTyp
};
#ifdef INT64
#define int_ot  int64_t
#define uint_ot uint64_t
#else
#define int_ot  int
#define uint_ot size_t
#endif

#ifdef REAL32
#define real_ot float
#else
#define real_ot double
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  int64_t OTNewOcTree( int_ot, const real_ot *, const real_ot *, int_ot, const int_ot *, const int_ot *, int_ot,
                       const int_ot *, const int_ot *, int_ot, const int_ot *, const int_ot *, int_ot, const int_ot *,
                       const int_ot *, int_ot, const int_ot *, const int_ot *, int_ot, const int_ot *, const int_ot *,
                       int_ot, const int_ot *, const int_ot *, int_ot, int_ot );
  int64_t OTNewOcTreeFromSTL( int_ot, const real_ot *, const real_ot *, int_ot, int_ot );
  size_t OTFreeOcTree( int64_t );
  int_ot OTGetBoundingBox( int64_t, int_ot, int_ot, int_ot *, real_ot *, real_ot *, int_ot );
  int_ot OTGetNearest( int64_t, int_ot, real_ot *, real_ot *, real_ot, int_ot( void *, int_ot ), void *, int_ot );
  int_ot OTIntersectSurface( int64_t, real_ot *, real_ot *, real_ot *, real_ot, int_ot( void *, int_ot ), void *,
                             int_ot );
  int_ot OTIsInside( int64_t, real_ot *, real_ot *, int_ot );
  int_ot OTProjectVertex( int64_t, real_ot *, int_ot, int_ot, real_ot *, int_ot );
  int_ot OTCheckIntersections( int64_t, int_ot, int_ot * );

#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
  //#include "octree_mesh_impl.h"

#define MaxItmOct         255
#define MaxOctLvl         255
#define ItmPerBuc         20
#define MemBlkSiz         100000
#define TngFlg            1
#define AniFlg            2
#define MaxThr            256
#define MaxRayTri         100000
#define MIN( a, b )       ( ( a ) < ( b ) ? ( a ) : ( b ) )
#define MAX( a, b )       ( ( a ) > ( b ) ? ( a ) : ( b ) )
#define POW( a )          ( ( a ) * ( a ) )
#define CUB( a )          ( ( a ) * ( a ) * ( a ) )
#define BUC( i, j, k, l ) ( ( ( ( ( i ) << l ) + ( j ) ) << l ) + ( k ) )

  typedef struct
  {
    int_ot idx;
    real_ot crd[ 3 ];
  } VerSct;

  typedef struct
  {
    VerSct *ver[ 2 ];
    real_ot tng[ 3 ], siz;
    int_ot idx;
  } EdgSct;

  typedef struct
  {
    EdgSct edg[ 3 ];
    VerSct *ver[ 3 ];
    real_ot srf, nrm[ 3 ];
    int_ot idx;
    float ani;
  } TriSct;

  typedef struct
  {
    real_ot crd[ 3 ][ 3 ], siz[ 3 ], tng[ 3 ][ 3 ], srf, nrm[ 3 ];
  } StlSct;

  typedef struct
  {
    VerSct *ver[ 4 ];
    EdgSct edg[ 4 ];
    TriSct tri[ 2 ];
    real_ot nrm[ 3 ];
    int_ot idx;
  } QadSct;

  typedef struct
  {
    VerSct *ver[ 4 ];
    EdgSct edg[ 6 ];
    TriSct tri[ 4 ];
    int_ot idx;
    float ani;
  } TetSct;

  typedef struct
  {
    VerSct *ver[ 8 ];
    EdgSct edg[ 12 ];
    QadSct qad[ 6 ];
    int_ot idx;
  } HexSct;

  typedef struct LnkSctPtr
  {
    int_ot typ, idx;
    struct LnkSctPtr *nex;
  } LnkSct;

  typedef struct MemSctPtr
  {
    size_t siz;
    void *adr;
    struct MemSctPtr *nex;
  } MemSct;

  typedef struct
  {
    VerSct ver[ 8 ], BakVer[ 8 ];
    EdgSct edg, BakEdg;
    TriSct tri, BakTri;
    QadSct qad, BakQad;

    HexSct hex, BakHex;
    char *FlgTab;
    int_ot *TagTab, tag;
  } MshThrSct;

  typedef struct
  {
    MshThrSct *thr[ MaxThr ];
    size_t UsrSiz[ OTNmbTyp ], NmbItm[ OTNmbTyp ];
    real_ot aniso, eps;
    int_ot BasIdx, StlMod;
    char *UsrPtr[ OTNmbTyp ];
    StlSct *StlTab;

  } MshSct;

  typedef struct OctSctPtr
  {
#ifdef WITH_FAST_MODE
    real_ot MinCrd[ 3 ], MaxCrd[ 3 ];
#endif
    union
    {
      LnkSct *lnk;
      struct OctSctPtr *son;
    };
    unsigned char NmbVer, NmbEdg, NmbFac, NmbVol, lvl, sub, ani, MaxItm;

  } OctSct;

  typedef struct
  {
    OctSct *oct;
    int_ot idx;
    char pos[ 3 ];
  } BucSct;

  typedef struct
  {
    VerSct ver[ 8 ];
    HexSct hex;
    int_ot tag, *ThrTag;
    BucSct **ThrStk;
  } OtrThrSct;

  typedef struct
  {
    OtrThrSct *thr[ MaxThr ];
    int_ot MaxLvl, NmbFreOct, NmbOct, GrdLvl, NmbBuc, NmbThr;
    size_t MemUse;
    real_ot eps, MaxSiz, MinSiz, BucSiz, bnd[ 2 ][ 3 ];
    OctSct oct, *CurOctBlk;
    BucSct *grd;
    LnkSct *NexFreLnk;
    MemSct *NexMem;
    MshSct *msh;
  } OtrSct;

  static void SetMshBox( OtrSct *, MshSct * );
  static void AddVer( MshSct *, OtrSct *, OctSct *, real_ot *, real_ot * );
  static void AddEdg( MshSct *, OtrSct *, OctSct *, real_ot *, real_ot * );
  static void AddTri( MshSct *, OtrSct *, OctSct *, real_ot *, real_ot * );
  static void AddQad( MshSct *, OtrSct *, OctSct *, real_ot *, real_ot * );

  static void SubOct( MshSct *, OtrSct *, OctSct *, real_ot *, real_ot * );
  static void LnkItm( OtrSct *, OctSct *, int_ot, int_ot, unsigned char );
  static OctSct *GetCrd( OctSct *, int_ot, real_ot *, real_ot *, real_ot * );
  static void GetBox( OtrSct *, OctSct *, int_ot, int_ot *, int_ot, int_ot *, char *, real_ot[ 2 ][ 3 ], real_ot,
                      real_ot *, real_ot *, int_ot );
  static int_ot BoxIntBox( real_ot[ 2 ][ 3 ], real_ot[ 2 ][ 3 ], real_ot );
  static void SetItm( MshSct *, int_ot, int_ot, int_ot, int_ot );
  static void SetSonCrd( int_ot, real_ot *, real_ot *, real_ot *, real_ot * );
  static void GetOctLnk( MshSct *, int_ot, real_ot *, int_ot *, real_ot *, OctSct *, real_ot *, real_ot *,
                         int_ot( void *, int_ot ), void *, int_ot );
  static void IntLinOct( OtrSct *, MshSct *, real_ot *, real_ot *, int_ot *, real_ot *, OctSct *, real_ot *, real_ot *,
                         int_ot( void *, int_ot ), void *, int_ot );
  static void IntVecOct( OtrSct *, MshSct *, real_ot *, real_ot *, int_ot *, int_ot *, OctSct *, real_ot *, real_ot *,
                         int_ot );
  static void GetBucBox( OtrSct *, BucSct *, real_ot *, real_ot * );
  static BucSct *GetBucNgb( OtrSct *, BucSct *, int_ot );
  static real_ot DisVerOct( real_ot *, real_ot *, real_ot * );
  static int_ot VerInsOct( real_ot *, real_ot *, real_ot * );
  static char *GetPtrItm( MshSct *, int_ot, int_ot );
  static void BakMshItm( MshSct * );
  static void RstMshItm( MshSct * );
#ifdef WITH_FAST_MODE
  static void CpyOctCrd( OctSct *, real_ot *, real_ot * );
#endif
  static void ChkTriIntTri( OtrSct *, OctSct *, int_ot *, int_ot, int_ot * );

  static int_ot EdgIntEdg( EdgSct *, EdgSct *, VerSct *, real_ot );
  static real_ot DisVerTri( MshSct *, real_ot *, TriSct * );
  static real_ot DisVerQad( MshSct *, real_ot *, QadSct * );
  static real_ot DisVerTet( MshSct *, real_ot *, TetSct * );
  static real_ot GetTriSrf( TriSct * );
  static real_ot GetVolTet( TetSct * );
  static real_ot DisVerEdg( real_ot *, EdgSct * );
  static void GetTriVec( TriSct *, real_ot * );
  static void SetTriNrm( TriSct * );
  static void SetTmpHex( HexSct *, real_ot *, real_ot * );
  static int_ot VerInsTet( VerSct *, TetSct *, real_ot );
  static int_ot VerInsHex( VerSct *, HexSct * );
  static int_ot EdgIntHex( EdgSct *, HexSct *, real_ot );
  static int_ot TriIntHex( TriSct *, HexSct *, real_ot );
  static int_ot QadIntHex( QadSct *, HexSct *, real_ot );
  static int_ot TetIntHex( TetSct *, HexSct *, real_ot );
  static int_ot EdgIntQad( HexSct *, int_ot, EdgSct *, VerSct *, real_ot );
  static int_ot EdgIntTri( TriSct *, EdgSct *, VerSct *, real_ot );
  static int_ot VerInsTri( TriSct *, VerSct *, real_ot );
  static int_ot VerInsEdg( EdgSct *, VerSct *, real_ot );
  static void SetEdgTng( EdgSct * );
  static real_ot GetTriAni( TriSct * );
#ifdef WITH_FAST_MODE
  static real_ot DisVerEdgStl( real_ot *, real_ot *, real_ot *, real_ot *, real_ot );
  static real_ot DisVerTriStl( MshSct *, real_ot *, StlSct * );
#endif

  static void PrjVerLin( real_ot *, real_ot *, real_ot *, real_ot * );
  static real_ot PrjVerPla( real_ot *, real_ot *, real_ot *, real_ot * );
  static void LinCmbVec3( real_ot, real_ot *, real_ot, real_ot *, real_ot * );
  static void ClrVec( real_ot * );
  static void CpyVec( real_ot *, real_ot * );
  static void AddVec2( real_ot *, real_ot * );
  static void SubVec3( real_ot *, real_ot *, real_ot * );
  static void AddScaVec1( real_ot, real_ot * );
  static void AddScaVec2( real_ot, real_ot *, real_ot * );
  static void MulVec1( real_ot, real_ot * );
  static void MulVec2( real_ot, real_ot *, real_ot * );

  static void CrsPrd( real_ot *, real_ot *, real_ot * );
  static real_ot DotPrd( real_ot *, real_ot * );
  static real_ot dis( real_ot *, real_ot * );
  static real_ot DisPow( real_ot *, real_ot * );
  static real_ot DisVerPla( real_ot *, real_ot *, real_ot * );
  static real_ot GetNrmVec( real_ot * );
  static real_ot VerInsBox( real_ot *, real_ot *, real_ot *, real_ot );
  static int_ot LinIntBox( real_ot *, real_ot *, real_ot *, real_ot *, real_ot );
  static void LinIntPla( real_ot *, real_ot *, real_ot *, real_ot *, real_ot * );

  static void *NewMem( OtrSct *, size_t );
  static void FreAllMem( OtrSct * );

  static const int_ot TetEdg[ 6 ][ 2 ]    = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
  static const int_ot TetFac[ 4 ][ 3 ]    = { { 1, 2, 3 }, { 2, 0, 3 }, { 3, 0, 1 }, { 0, 2, 1 } };
  static const int_ot TetFacEdg[ 4 ][ 3 ] = { { 5, 4, 3 }, { 2, 5, 1 }, { 0, 4, 2 }, { 3, 1, 0 } };
  static const int_ot tvpe[ 12 ][ 2 ]     = { { 3, 2 }, { 0, 1 }, { 4, 5 }, { 7, 6 }, { 3, 7 }, { 2, 6 },
                                              { 1, 5 }, { 0, 4 }, { 3, 0 }, { 7, 4 }, { 6, 5 }, { 2, 1 } };
  static const int_ot tvpf[ 6 ][ 4 ]      = { { 3, 0, 4, 7 }, { 5, 1, 2, 6 }, { 3, 2, 1, 0 },
                                              { 5, 6, 7, 4 }, { 3, 7, 6, 2 }, { 5, 4, 0, 1 } };

  int64_t OTNewOcTree( int_ot NmbVer, const real_ot *PtrCrd1, const real_ot *PtrCrd2, int_ot NmbEdg,
                       const int_ot *PtrEdg1, const int_ot *PtrEdg2, int_ot NmbTri, const int_ot *PtrTri1,
                       const int_ot *PtrTri2, int_ot NmbQad, const int_ot *PtrQad1, const int_ot *PtrQad2,
                       int_ot NmbTet, const int_ot *PtrTet1, const int_ot *PtrTet2, int_ot NmbPyr,
                       const int_ot *PtrPyr1, const int_ot *PtrPyr2, int_ot NmbPri, const int_ot *PtrPri1,
                       const int_ot *PtrPri2, int_ot NmbHex, const int_ot *PtrHex1, const int_ot *PtrHex2,
                       int_ot BasIdx, int_ot NmbThr )
  {
    int_ot i, j, k, t, EdgIdx, MaxItmCnt, TotItmCnt = 0, idx = 0;
    real_ot crd[ 3 ];
    BucSct *buc;
    OtrSct *otr = NULL;
    MshSct *msh;
    OtrThrSct *OctThr;
    MshThrSct *MshThr;

    if ( !NmbVer || !PtrCrd1 || !PtrCrd2 || ( BasIdx < 0 ) || ( BasIdx > 1 ) ) return ( 0 );

    NmbThr = MAX( NmbThr, 1 );
    NmbThr = MIN( NmbThr, MaxThr );

    MaxItmCnt = NmbVer;

    otr = (OtrSct *)calloc( 1, sizeof( OtrSct ) );
    assert( otr );

    msh         = (MshSct *)NewMem( otr, sizeof( MshSct ) );
    msh->BasIdx = BasIdx;

    if ( NmbVer < 0 )
    {
      msh->StlMod = 1;
      NmbTri      = -NmbVer;
      PtrTri1     = (int_ot *)PtrCrd1;
      PtrTri2     = (int_ot *)PtrCrd2;
      NmbVer      = 0;
      PtrCrd1     = NULL;
      PtrCrd2     = NULL;
    }
    else
      msh->StlMod = 0;

    msh->NmbItm[ OTTypVer ] = NmbVer;
    TotItmCnt += NmbVer;
    msh->UsrPtr[ OTTypVer ] = (char *)PtrCrd1;
    msh->UsrSiz[ OTTypVer ] = (char *)PtrCrd2 - (char *)PtrCrd1;

    msh->NmbItm[ OTTypEdg ] = NmbEdg;
    MaxItmCnt               = MAX( MaxItmCnt, NmbEdg );
    TotItmCnt += NmbEdg;
    msh->UsrPtr[ OTTypEdg ] = (char *)PtrEdg1;
    msh->UsrSiz[ OTTypEdg ] = (char *)PtrEdg2 - (char *)PtrEdg1;

    msh->NmbItm[ OTTypTri ] = NmbTri;
    MaxItmCnt               = MAX( MaxItmCnt, NmbTri );
    TotItmCnt += NmbTri;
    msh->UsrPtr[ OTTypTri ] = (char *)PtrTri1;
    msh->UsrSiz[ OTTypTri ] = (char *)PtrTri2 - (char *)PtrTri1;

    msh->NmbItm[ OTTypQad ] = NmbQad;
    MaxItmCnt               = MAX( MaxItmCnt, NmbQad );
    TotItmCnt += NmbQad;
    msh->UsrPtr[ OTTypQad ] = (char *)PtrQad1;
    msh->UsrSiz[ OTTypQad ] = (char *)PtrQad2 - (char *)PtrQad1;

    for ( t = 0; t < NmbThr; t++ )
    {
      MshThr = msh->thr[ t ] = (MshThrSct *)NewMem( otr, sizeof( MshThrSct ) );
      memset( MshThr, 0, sizeof( MshThrSct ) );

      MshThr->FlgTab = (char *)NewMem( otr, ( MaxItmCnt + 1 ) * sizeof( char ) );
      memset( MshThr->FlgTab, 0, ( MaxItmCnt + 1 ) * sizeof( char ) );

      MshThr->TagTab = (int_ot *)NewMem( otr, ( MaxItmCnt + 1 ) * sizeof( int_ot ) );
      memset( MshThr->TagTab, 0, ( MaxItmCnt + 1 ) * sizeof( int_ot ) );

      MshThr->tag = 0;

      otr->thr[ t ] = (OtrThrSct *)NewMem( otr, sizeof( OtrThrSct ) );
      memset( otr->thr[ t ], 0, sizeof( OtrThrSct ) );
    }

    SetMshBox( otr, msh );
    otr->msh = msh;
    msh->eps = otr->eps;

    // if ( TotItmCnt >= ItmPerBuc )
    //   otr->GrdLvl = 10;  //(int)( log( TotItmCnt / ItmPerBuc ) / ( 3 * log( 2 ) ) );
    // else
    otr->GrdLvl = 0;

    otr->NmbBuc = 1 << otr->GrdLvl;
    otr->grd    = (BucSct *)NewMem( otr, CUB( otr->NmbBuc ) * sizeof( BucSct ) );
    otr->NmbThr = NmbThr;

    // printf("[debug ]%d \n",)

    for ( t = 0; t < NmbThr; t++ )
    {
      MshThr = msh->thr[ t ];
      OctThr = otr->thr[ t ];

      OctThr->ThrStk = (BucSct **)NewMem( otr, CUB( otr->NmbBuc ) * sizeof( void * ) );
      OctThr->ThrTag = (int_ot *)NewMem( otr, CUB( otr->NmbBuc ) * sizeof( int_ot ) );

      for ( i = 0; i < 2; i++ ) MshThr->edg.ver[ i ] = &MshThr->ver[ i ];

      for ( i = 0; i < 3; i++ )
      {
        MshThr->tri.ver[ i ]          = &MshThr->ver[ i ];
        MshThr->tri.edg[ i ].ver[ 0 ] = &MshThr->ver[ ( i + 1 ) % 3 ];
        MshThr->tri.edg[ i ].ver[ 1 ] = &MshThr->ver[ ( i + 2 ) % 3 ];
      }

      for ( i = 0; i < 4; i++ )
      {
        MshThr->qad.ver[ i ]          = &MshThr->ver[ i ];
        MshThr->qad.edg[ i ].ver[ 0 ] = &MshThr->ver[ i ];
        MshThr->qad.edg[ i ].ver[ 1 ] = &MshThr->ver[ ( i + 1 ) % 4 ];
      }

      MshThr->qad.tri[ 0 ].ver[ 0 ] = &MshThr->ver[ 0 ];
      MshThr->qad.tri[ 0 ].ver[ 1 ] = &MshThr->ver[ 1 ];
      MshThr->qad.tri[ 0 ].ver[ 2 ] = &MshThr->ver[ 2 ];

      MshThr->qad.tri[ 1 ].ver[ 0 ] = &MshThr->ver[ 3 ];
      MshThr->qad.tri[ 1 ].ver[ 1 ] = &MshThr->ver[ 2 ];
      MshThr->qad.tri[ 1 ].ver[ 2 ] = &MshThr->ver[ 1 ];

      for ( i = 0; i < 2; i++ )
        for ( j = 0; j < 3; j++ )
        {
          MshThr->qad.tri[ i ].edg[ j ].ver[ 0 ] = MshThr->qad.tri[ i ].ver[ ( j + 1 ) % 3 ];
          MshThr->qad.tri[ i ].edg[ j ].ver[ 1 ] = MshThr->qad.tri[ i ].ver[ ( j + 2 ) % 3 ];
        }

      for ( i = 0; i < 8; i++ ) OctThr->hex.ver[ i ] = &OctThr->ver[ i ];

      for ( i = 0; i < 12; i++ )
      {
        OctThr->hex.edg[ i ].ver[ 0 ] = &OctThr->ver[ tvpe[ i ][ 0 ] ];
        OctThr->hex.edg[ i ].ver[ 1 ] = &OctThr->ver[ tvpe[ i ][ 1 ] ];
      }

      for ( i = 0; i < 6; i++ )
        for ( j = 0; j < 4; j++ ) OctThr->hex.qad[ i ].ver[ j ] = &OctThr->ver[ tvpf[ i ][ j ] ];

      for ( i = 0; i < 4; i++ )
      {
        OctThr->hex.edg[ i ].tng[ 0 ] = 1;
        OctThr->hex.edg[ i ].tng[ 1 ] = 0;
        OctThr->hex.edg[ i ].tng[ 2 ] = 0;

        OctThr->hex.edg[ i + 4 ].tng[ 0 ] = 0;
        OctThr->hex.edg[ i + 4 ].tng[ 1 ] = 1;
        OctThr->hex.edg[ i + 4 ].tng[ 2 ] = 0;

        OctThr->hex.edg[ i + 8 ].tng[ 0 ] = 0;
        OctThr->hex.edg[ i + 8 ].tng[ 1 ] = 0;
        OctThr->hex.edg[ i + 8 ].tng[ 2 ] = 1;
      }

      OctThr->hex.qad[ 0 ].nrm[ 0 ] = 1;
      OctThr->hex.qad[ 0 ].nrm[ 1 ] = 0;
      OctThr->hex.qad[ 0 ].nrm[ 2 ] = 0;

      OctThr->hex.qad[ 1 ].nrm[ 0 ] = -1;
      OctThr->hex.qad[ 1 ].nrm[ 1 ] = 0;
      OctThr->hex.qad[ 1 ].nrm[ 2 ] = 0;

      OctThr->hex.qad[ 2 ].nrm[ 0 ] = 0;
      OctThr->hex.qad[ 2 ].nrm[ 1 ] = 1;
      OctThr->hex.qad[ 2 ].nrm[ 2 ] = 0;

      OctThr->hex.qad[ 3 ].nrm[ 0 ] = 0;
      OctThr->hex.qad[ 3 ].nrm[ 1 ] = -1;
      OctThr->hex.qad[ 3 ].nrm[ 2 ] = 0;

      OctThr->hex.qad[ 4 ].nrm[ 0 ] = 0;
      OctThr->hex.qad[ 4 ].nrm[ 1 ] = 0;
      OctThr->hex.qad[ 4 ].nrm[ 2 ] = 1;

      OctThr->hex.qad[ 5 ].nrm[ 0 ] = 0;
      OctThr->hex.qad[ 5 ].nrm[ 1 ] = 0;
      OctThr->hex.qad[ 5 ].nrm[ 2 ] = -1;
    }

    for ( i = 0; i < msh->NmbItm[ OTTypVer ]; i++ )
    {
      SetItm( msh, OTTypVer, i + BasIdx, 0, 0 );
      AddVer( msh, otr, &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );
    }

    for ( i = 0; i < msh->NmbItm[ OTTypEdg ]; i++ )
    {
      SetItm( msh, OTTypEdg, i + BasIdx, 0, 0 );
      AddEdg( msh, otr, &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );
    }

#ifdef WITH_FAST_MODE
    if ( msh->NmbItm[ OTTypTri ] )
    {
      msh->StlTab = NewMem( otr, ( msh->NmbItm[ OTTypTri ] + 1 ) * sizeof( StlSct ) );
      MshThr      = msh->thr[ 0 ];

      for ( i = 0; i < msh->NmbItm[ OTTypTri ]; i++ )
      {
        SetItm( msh, OTTypTri, i + BasIdx, TngFlg | AniFlg, 0 );
        AddTri( msh, otr, &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );

        CpyVec( MshThr->tri.ver[ 0 ]->crd, msh->StlTab[ i + 1 ].crd[ 0 ] );
        CpyVec( MshThr->tri.ver[ 1 ]->crd, msh->StlTab[ i + 1 ].crd[ 1 ] );
        CpyVec( MshThr->tri.ver[ 2 ]->crd, msh->StlTab[ i + 1 ].crd[ 2 ] );
        CpyVec( MshThr->tri.nrm, msh->StlTab[ i + 1 ].nrm );
        CpyVec( MshThr->tri.edg[ 0 ].tng, msh->StlTab[ i + 1 ].tng[ 0 ] );
        CpyVec( MshThr->tri.edg[ 1 ].tng, msh->StlTab[ i + 1 ].tng[ 1 ] );
        CpyVec( MshThr->tri.edg[ 2 ].tng, msh->StlTab[ i + 1 ].tng[ 2 ] );
        msh->StlTab[ i + 1 ].siz[ 0 ] = MshThr->tri.edg[ 0 ].siz;
        msh->StlTab[ i + 1 ].siz[ 1 ] = MshThr->tri.edg[ 1 ].siz;
        msh->StlTab[ i + 1 ].siz[ 2 ] = MshThr->tri.edg[ 2 ].siz;
        msh->StlTab[ i + 1 ].srf      = MshThr->tri.srf;
      }
    }
#else
  for ( i = 0; i < msh->NmbItm[ OTTypTri ]; i++ )
  {
    SetItm( msh, OTTypTri, i + BasIdx, TngFlg | AniFlg, 0 );
    AddTri( msh, otr, &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );
  }
#endif

    for ( i = 0; i < msh->NmbItm[ OTTypQad ]; i++ )
    {
      SetItm( msh, OTTypQad, i + BasIdx, TngFlg | AniFlg, 0 );
      AddQad( msh, otr, &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );
    }

    otr->BucSiz = ( otr->bnd[ 1 ][ 0 ] - otr->bnd[ 0 ][ 0 ] ) / (real_ot)otr->NmbBuc;
    crd[ 0 ]    = otr->bnd[ 0 ][ 0 ] + otr->BucSiz / 2.;

    for ( i = 0; i < otr->NmbBuc; i++ )
    {
      crd[ 1 ] = otr->bnd[ 0 ][ 1 ] + otr->BucSiz / 2.;

      for ( j = 0; j < otr->NmbBuc; j++ )
      {
        crd[ 2 ] = otr->bnd[ 0 ][ 2 ] + otr->BucSiz / 2.;

        for ( k = 0; k < otr->NmbBuc; k++ )
        {
          buc           = &otr->grd[ BUC( i, j, k, otr->GrdLvl ) ];
          buc->oct      = GetCrd( &otr->oct, otr->GrdLvl, crd, otr->bnd[ 0 ], otr->bnd[ 1 ] );
          buc->pos[ 0 ] = i;
          buc->pos[ 1 ] = j;
          buc->pos[ 2 ] = k;
          buc->idx      = idx++;
          crd[ 2 ] += otr->BucSiz;
        }

        crd[ 1 ] += otr->BucSiz;
      }

      crd[ 0 ] += otr->BucSiz;
    }

#ifdef WITH_FAST_MODE
    CpyOctCrd( &otr->oct, otr->bnd[ 0 ], otr->bnd[ 1 ] );
#endif

    return ( (int64_t)otr );
  }

  int64_t OTNewOcTreeFromSTL( int_ot NmbTri, const real_ot *PtrCrd1, const real_ot *PtrCrd2, int_ot BasIdx,
                              int_ot NmbThr )
  {
    return ( OTNewOcTree( -NmbTri, PtrCrd1, PtrCrd2, 0, NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, 0,
                          NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, BasIdx, NmbThr ) );
  }

  size_t OTFreeOcTree( int64_t OctIdx )
  {
    OtrSct *otr   = (OtrSct *)OctIdx;
    size_t MemUse = otr->MemUse;

    FreAllMem( otr );
    free( otr );

    return ( MemUse );
  }

  int_ot OTGetBoundingBox( int64_t OctIdx, int_ot typ, int_ot MaxItm, int_ot *ItmTab, real_ot MinCrd[ 3 ],
                           real_ot MaxCrd[ 3 ], int_ot ThrIdx )
  {
    int_ot i, NmbItm = 0;
    real_ot box[ 2 ][ 3 ] = { { MinCrd[ 0 ], MinCrd[ 1 ], MinCrd[ 2 ] }, { MaxCrd[ 0 ], MaxCrd[ 1 ], MaxCrd[ 2 ] } };
    OtrSct *otr           = (OtrSct *)OctIdx;

    GetBox( otr, &otr->oct, typ, &NmbItm, MaxItm, ItmTab, otr->msh->thr[ ThrIdx ]->FlgTab, box, otr->eps, otr->bnd[ 0 ],
            otr->bnd[ 1 ], ThrIdx );

    for ( i = 0; i < NmbItm; i++ ) otr->msh->thr[ ThrIdx ]->FlgTab[ ItmTab[ i ] ] = 0;

    return ( NmbItm );
  }

  int_ot OTCheckIntersections( int64_t OctIdx, int_ot MaxItm, int_ot *ItmTab )
  {
    int_ot NmbItm = 0;
    OtrSct *otr   = (OtrSct *)OctIdx;

    ChkTriIntTri( otr, &otr->oct, &NmbItm, MaxItm, ItmTab );

    return ( NmbItm );
  }

  static void ChkTriIntTri( OtrSct *otr, OctSct *oct, int_ot *NmbItm, int_ot MaxItm, int_ot *ItmTab )
  {
    int_ot i, j, NmbTri = 0, *IdxTab;
    LnkSct *lnk;
    VerSct IntVer, VerTab[ 3 * MaxItmOct * 10 ];
    TriSct *tri, TriTab[ MaxItmOct * 10 ];
    MshThrSct *ThrMsh = otr->msh->thr[ 0 ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ ) ChkTriIntTri( otr, oct->son + i, NmbItm, MaxItm, ItmTab );
    }
    else if ( ( lnk = oct->lnk ) && ( *NmbItm < MaxItm ) )
    {
      do
      {
        if ( lnk->typ == OTTypTri )
        {
          SetItm( otr->msh, OTTypTri, lnk->idx, TngFlg, 0 );

          tri      = &TriTab[ NmbTri ];
          tri->idx = lnk->idx;
          CpyVec( ThrMsh->tri.nrm, tri->nrm );
          tri->srf = ThrMsh->tri.srf;
          IdxTab   = (int_ot *)GetPtrItm( otr->msh, OTTypTri, lnk->idx );

          for ( i = 0; i < 3; i++ )
          {
            tri->ver[ i ]      = &VerTab[ 3 * NmbTri + i ];
            tri->ver[ i ]->idx = IdxTab[ i ];
            CpyVec( ThrMsh->ver[ i ].crd, tri->ver[ i ]->crd );
          }

          for ( i = 0; i < 3; i++ )
          {
            tri->edg[ i ].ver[ 0 ] = tri->ver[ ( i + 1 ) % 3 ];
            tri->edg[ i ].ver[ 1 ] = tri->ver[ ( i + 2 ) % 3 ];
            CpyVec( ThrMsh->tri.edg[ i ].tng, tri->edg[ i ].tng );
            tri->edg[ i ].siz = ThrMsh->tri.edg[ i ].siz;
          }

          NmbTri++;
        }
      } while ( ( lnk = lnk->nex ) );

      for ( i = 0; i < NmbTri; i++ )
      {
        for ( j = 0; j < NmbTri; j++ )
        {
          if ( ( TriTab[ i ].ver[ 0 ]->idx == TriTab[ j ].ver[ 0 ]->idx ) ||
               ( TriTab[ i ].ver[ 0 ]->idx == TriTab[ j ].ver[ 1 ]->idx ) ||
               ( TriTab[ i ].ver[ 0 ]->idx == TriTab[ j ].ver[ 2 ]->idx ) ||
               ( TriTab[ i ].ver[ 1 ]->idx == TriTab[ j ].ver[ 0 ]->idx ) ||
               ( TriTab[ i ].ver[ 1 ]->idx == TriTab[ j ].ver[ 1 ]->idx ) ||
               ( TriTab[ i ].ver[ 1 ]->idx == TriTab[ j ].ver[ 2 ]->idx ) ||
               ( TriTab[ i ].ver[ 2 ]->idx == TriTab[ j ].ver[ 0 ]->idx ) ||
               ( TriTab[ i ].ver[ 2 ]->idx == TriTab[ j ].ver[ 1 ]->idx ) ||
               ( TriTab[ i ].ver[ 2 ]->idx == TriTab[ j ].ver[ 2 ]->idx ) )
          {
            continue;
          }

          if ( EdgIntTri( &TriTab[ i ], &TriTab[ j ].edg[ 0 ], &IntVer, otr->eps ) ||
               EdgIntTri( &TriTab[ i ], &TriTab[ j ].edg[ 1 ], &IntVer, otr->eps ) ||
               EdgIntTri( &TriTab[ i ], &TriTab[ j ].edg[ 2 ], &IntVer, otr->eps ) )
          {
            ItmTab[ *NmbItm ] = TriTab[ i ].idx;
            ( *NmbItm )++;

            if ( *NmbItm >= MaxItm ) return;

            break;
          }
        }
      }
    }
  }

  static OctSct *GetCrd( OctSct *oct, int_ot MaxLvl, real_ot VerCrd[ 3 ], real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot SonIdx;
    real_ot MidCrd[ 3 ], OctMin[ 3 ], OctMax[ 3 ], SonMin[ 3 ], SonMax[ 3 ];

    CpyVec( MinCrd, OctMin );
    CpyVec( MaxCrd, OctMax );

    while ( oct->sub && ( oct->lvl < MaxLvl ) )
    {
      LinCmbVec3( .5, OctMin, .5, OctMax, MidCrd );

      SonIdx = ( ( VerCrd[ 0 ] < MidCrd[ 0 ] ) ? 0 : 1 ) | ( ( VerCrd[ 1 ] < MidCrd[ 1 ] ) ? 0 : 2 ) |
               ( ( VerCrd[ 2 ] < MidCrd[ 2 ] ) ? 0 : 4 );

      SetSonCrd( SonIdx, SonMin, SonMax, OctMin, OctMax );
      CpyVec( SonMin, OctMin );
      CpyVec( SonMax, OctMax );
      oct = oct->son + SonIdx;
    }

    return ( oct );
  }

  static void GetBox( OtrSct *otr, OctSct *oct, int_ot typ, int_ot *NmbItm, int_ot MaxItm, int_ot *ItmTab, char *FlgTab,
                      real_ot box[ 2 ][ 3 ], real_ot eps, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ], int_ot ThrIdx )
  {
    int_ot i;
    LnkSct *lnk;
    OtrThrSct *ThrOct          = otr->thr[ ThrIdx ];
    MshThrSct *ThrMsh          = otr->msh->thr[ ThrIdx ];
    real_ot xmid               = ( MinCrd[ 0 ] + MaxCrd[ 0 ] ) / 2.;
    real_ot ymid               = ( MinCrd[ 1 ] + MaxCrd[ 1 ] ) / 2.;
    real_ot zmid               = ( MinCrd[ 2 ] + MaxCrd[ 2 ] ) / 2.;
    real_ot son[ 8 ][ 2 ][ 3 ] = { { { MinCrd[ 0 ], MinCrd[ 1 ], MinCrd[ 2 ] }, { xmid, ymid, zmid } },
                                   { { xmid, MinCrd[ 1 ], MinCrd[ 2 ] }, { MaxCrd[ 0 ], ymid, zmid } },
                                   { { MinCrd[ 0 ], ymid, MinCrd[ 2 ] }, { xmid, MaxCrd[ 1 ], zmid } },
                                   { { xmid, ymid, MinCrd[ 2 ] }, { MaxCrd[ 0 ], MaxCrd[ 1 ], zmid } },
                                   { { MinCrd[ 0 ], MinCrd[ 1 ], zmid }, { xmid, ymid, MaxCrd[ 2 ] } },
                                   { { xmid, MinCrd[ 1 ], zmid }, { MaxCrd[ 0 ], ymid, MaxCrd[ 2 ] } },
                                   { { MinCrd[ 0 ], ymid, zmid }, { xmid, MaxCrd[ 1 ], MaxCrd[ 2 ] } },
                                   { { xmid, ymid, zmid }, { MaxCrd[ 0 ], MaxCrd[ 1 ], MaxCrd[ 2 ] } } };

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
        if ( BoxIntBox( box, son[ i ], eps ) )
          GetBox( otr, oct->son + i, typ, NmbItm, MaxItm, ItmTab, FlgTab, box, eps, son[ i ][ 0 ], son[ i ][ 1 ],
                  ThrIdx );
    }
    else if ( ( lnk = oct->lnk ) && ( *NmbItm < MaxItm ) )
    {
      SetTmpHex( &ThrOct->hex, box[ 0 ], box[ 1 ] );

      do
      {
        if ( lnk->typ != typ ) continue;

        if ( lnk->typ == OTTypVer )
        {
          SetItm( otr->msh, OTTypVer, lnk->idx, 0, ThrIdx );

          if ( !VerInsHex( &ThrMsh->ver[ 0 ], &ThrOct->hex ) ) continue;
        }
        else if ( lnk->typ == OTTypEdg )
        {
          SetItm( otr->msh, OTTypEdg, lnk->idx, 0, ThrIdx );

          if ( !EdgIntHex( &ThrMsh->edg, &ThrOct->hex, otr->eps ) ) continue;
        }
        else if ( lnk->typ == OTTypTri )
        {
          SetItm( otr->msh, OTTypTri, lnk->idx, TngFlg, ThrIdx );

          if ( !TriIntHex( &ThrMsh->tri, &ThrOct->hex, otr->eps ) ) continue;
        }
        else if ( lnk->typ == OTTypQad )
        {
          SetItm( otr->msh, OTTypQad, lnk->idx, TngFlg, ThrIdx );

          if ( !QadIntHex( &ThrMsh->qad, &ThrOct->hex, otr->eps ) ) continue;
        }

        if ( !FlgTab[ lnk->idx ] )
        {
          ItmTab[ ( *NmbItm )++ ] = lnk->idx;
          FlgTab[ lnk->idx ]      = 1;
        }
      } while ( ( lnk = lnk->nex ) && ( *NmbItm < MaxItm ) );
    }
  }

  int_ot OTGetNearest( int64_t OctIdx, int_ot typ, real_ot *VerCrd, real_ot *MinDis, real_ot MaxDis,
                       int_ot( UsrPrc )( void *, int_ot ), void *UsrDat, int_ot ThrIdx )
  {
    OtrSct *otr = (OtrSct *)OctIdx;
    int_ot i, ins = 0, out = 0, MinItm = -1, ini[ 3 ], *tag, len;
    real_ot MinCrd[ 3 ], MaxCrd[ 3 ];
    MshSct *msh = otr->msh;
    BucSct *IniBuc, *buc, *ngb, **stk;
    OtrThrSct *ThrOct = otr->thr[ ThrIdx ];
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];

    if ( ( ThrIdx < 0 ) || ( ThrIdx >= otr->NmbThr ) )
    {
      // printf( "return0\n" );
      return ( 0 );
    }

    ThrOct->tag++;
    ThrMsh->tag = ThrOct->tag;
    tag         = ThrOct->ThrTag;
    stk         = ThrOct->ThrStk;
    len         = otr->NmbBuc;

    if ( MaxDis > 0. )
      *MinDis = POW( MaxDis );
    else
      *MinDis = DBL_MAX;

    for ( i = 0; i < 3; i++ )
    {
      ini[ i ] = (int)( ( VerCrd[ i ] - otr->bnd[ 0 ][ i ] ) / otr->BucSiz );
      ini[ i ] = MAX( 0, ini[ i ] );
      ini[ i ] = MIN( len - 1, ini[ i ] );
    }

    IniBuc = &otr->grd[ BUC( ini[ 0 ], ini[ 1 ], ini[ 2 ], otr->GrdLvl ) ];

    stk[ ins++ ]       = IniBuc;
    tag[ IniBuc->idx ] = ThrOct->tag;

    while ( ins > out )
    {
      buc = stk[ out++ ];
      GetBucBox( otr, buc, MinCrd, MaxCrd );
      GetOctLnk( msh, typ, VerCrd, &MinItm, MinDis, buc->oct, MinCrd, MaxCrd, UsrPrc, UsrDat, ThrIdx );

      for ( i = 0; i < 6; i++ )
      {
        if ( !( ngb = GetBucNgb( otr, buc, i ) ) || ( tag[ ngb->idx ] == ThrOct->tag ) ) continue;

        GetBucBox( otr, ngb, MinCrd, MaxCrd );

        if ( DisVerOct( VerCrd, MinCrd, MaxCrd ) < *MinDis )
        {
          stk[ ins++ ]    = ngb;
          tag[ ngb->idx ] = ThrOct->tag;
        }
      }
    }

    *MinDis = sqrt( *MinDis );

    return ( MinItm );
  }

  int_ot OTIntersectSurface( int64_t OctIdx, real_ot *VerCrd, real_ot *VerTng, real_ot *MinDis, real_ot MaxDis,
                             int_ot( UsrPrc )( void *, int_ot ), void *UsrDat, int_ot ThrIdx )
  {
    // OtrSct *otr       = (OtrSct *)OctIdx;
    // OtrThrSct *ThrOct = otr->thr[ ThrIdx ];
    // MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];
    // int_ot i, ins = 0, out = 0, MinItm = -1, ini[ 3 ], *tag, len;
    // real_ot MinCrd[ 3 ], MaxCrd[ 3 ];
    // MshSct *msh = otr->msh;
    // BucSct *IniBuc, *buc, *ngb, **stk;

    // ThrOct->tag++;
    // ThrMsh->tag = ThrOct->tag;
    // tag         = ThrOct->ThrTag;
    // stk         = ThrOct->ThrStk;
    // len         = otr->NmbBuc;
    // *MinDis     = ( MaxDis > 0. ) ? POW( MaxDis ) : DBL_MAX;

    // for ( i = 0; i < 3; i++ )
    // {
    //   ini[ i ] = (int)( ( VerCrd[ i ] - otr->bnd[ 0 ][ i ] ) / otr->BucSiz );
    //   ini[ i ] = MAX( 0, ini[ i ] );
    //   ini[ i ] = MIN( otr->NmbBuc - 1, ini[ i ] );
    // }

    // IniBuc = &otr->grd[ BUC( ini[ 0 ], ini[ 1 ], ini[ 2 ], otr->GrdLvl ) ];

    // stk[ ins++ ]       = IniBuc;
    // tag[ IniBuc->idx ] = ThrOct->tag;

    // while ( ins > out )
    // {
    //   buc = stk[ out++ ];
    //   GetBucBox( otr, buc, MinCrd, MaxCrd );
    //   IntLinOct( otr, msh, VerCrd, VerTng, &MinItm, MinDis, buc->oct, MinCrd, MaxCrd, UsrPrc, UsrDat, ThrIdx );

    //   for ( i = 0; i < 6; i++ )
    //   {
    //     if ( !( ngb = GetBucNgb( otr, buc, i ) ) || ( tag[ ngb->idx ] == ThrOct->tag ) ) continue;

    //     GetBucBox( otr, ngb, MinCrd, MaxCrd );

    //     if ( !LinIntBox( VerCrd, VerTng, MinCrd, MaxCrd, otr->eps ) ) continue;

    //     if ( DisVerOct( VerCrd, MinCrd, MaxCrd ) < *MinDis )
    //     {
    //       stk[ ins++ ]    = ngb;
    //       tag[ ngb->idx ] = ThrOct->tag;
    //     }
    //   }
    // }

    // *MinDis = sqrt( *MinDis );

    // return ( MinItm );

    OtrSct *otr       = (OtrSct *)OctIdx;
    OtrThrSct *ThrOct = otr->thr[ ThrIdx ];
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];
    int_ot i, ins = 0, out = 0, MinItm = 0, ini[ 3 ], *tag;
    real_ot MinCrd[ 3 ], MaxCrd[ 3 ];
    MshSct *msh = otr->msh;
    BucSct *IniBuc, *buc, *ngb, **stk;

    ThrOct->tag++;
    ThrMsh->tag = ThrOct->tag;
    tag         = ThrOct->ThrTag;
    stk         = ThrOct->ThrStk;
    *MinDis     = ( MaxDis > 0. ) ? POW( MaxDis ) : DBL_MAX;

    // Get the vertex's integer coordinates in the grid
    // and clip it if it stands outside the bounding box
    for ( i = 0; i < 3; i++ )
    {
      ini[ i ] = (int)( ( VerCrd[ i ] - otr->bnd[ 0 ][ i ] ) / otr->BucSiz );
      ini[ i ] = MAX( 0, ini[ i ] );
      ini[ i ] = MIN( otr->NmbBuc - 1, ini[ i ] );
    }

    IniBuc = &otr->grd[ BUC( ini[ 0 ], ini[ 1 ], ini[ 2 ], otr->GrdLvl ) ];

    // Push the octant containing the starting point on the lifo stack
    stk[ ins++ ]       = IniBuc;
    tag[ IniBuc->idx ] = ThrOct->tag;

    // Flood fill processing of the grid :
    // check octant's contents distance against the closest item
    while ( ins > out )
    {
      buc = stk[ out++ ];
      GetBucBox( otr, buc, MinCrd, MaxCrd );
      IntLinOct( otr, msh, VerCrd, VerTng, &MinItm, MinDis, buc->oct, MinCrd, MaxCrd, UsrPrc, UsrDat, ThrIdx );

      // Push unprocessed neighbours intersected by the line
      // on the stack as long as they are not too far
      for ( i = 0; i < 6; i++ )
      {
        if ( !( ngb = GetBucNgb( otr, buc, i ) ) || ( tag[ ngb->idx ] == ThrOct->tag ) ) continue;

        GetBucBox( otr, ngb, MinCrd, MaxCrd );

        if ( !LinIntBox( VerCrd, VerTng, MinCrd, MaxCrd, otr->eps ) ) continue;

        if ( DisVerOct( VerCrd, MinCrd, MaxCrd ) < *MinDis )
        {
          stk[ ins++ ]    = ngb;
          tag[ ngb->idx ] = ThrOct->tag;
        }
      }
    }

    *MinDis = sqrt( *MinDis );

    return ( MinItm );
  }

  int_ot OTIsInside( int64_t OctIdx, real_ot *VerCrd, real_ot *VerTng, int_ot ThrIdx )
  {
    OtrSct *otr       = (OtrSct *)OctIdx;
    OtrThrSct *ThrOct = otr->thr[ ThrIdx ];
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];
    int_ot i, ins = 0, out = 0, ini[ 3 ], *tag, len;
    int_ot NmbTri = 0, TriTab[ MaxRayTri ];
    real_ot MinCrd[ 3 ], MaxCrd[ 3 ];
    MshSct *msh = otr->msh;
    BucSct *IniBuc, *buc, *ngb, **stk;

    ThrOct->tag++;
    ThrMsh->tag = ThrOct->tag;
    tag         = ThrOct->ThrTag;
    stk         = ThrOct->ThrStk;
    len         = otr->NmbBuc;

    for ( i = 0; i < 3; i++ )
    {
      ini[ i ] = (int)( ( VerCrd[ i ] - otr->bnd[ 0 ][ i ] ) / otr->BucSiz );
      ini[ i ] = MAX( 0, ini[ i ] );
      ini[ i ] = MIN( otr->NmbBuc - 1, ini[ i ] );
    }

    IniBuc = &otr->grd[ BUC( ini[ 0 ], ini[ 1 ], ini[ 2 ], otr->GrdLvl ) ];

    stk[ ins++ ]       = IniBuc;
    tag[ IniBuc->idx ] = ThrOct->tag;

    while ( ins > out )
    {
      buc = stk[ out++ ];
      GetBucBox( otr, buc, MinCrd, MaxCrd );
      IntVecOct( otr, msh, VerCrd, VerTng, &NmbTri, TriTab, buc->oct, MinCrd, MaxCrd, ThrIdx );

      for ( i = 0; i < 6; i++ )
      {
        if ( !( ngb = GetBucNgb( otr, buc, i ) ) || ( tag[ ngb->idx ] == ThrOct->tag ) ) continue;

        GetBucBox( otr, ngb, MinCrd, MaxCrd );

        if ( !LinIntBox( VerCrd, VerTng, MinCrd, MaxCrd, otr->eps ) ) continue;

        stk[ ins++ ]    = ngb;
        tag[ ngb->idx ] = ThrOct->tag;
      }
    }

    return ( NmbTri );
  }

  int_ot OTProjectVertex( int64_t OctIdx, real_ot *VerCrd, int_ot typ, int_ot MinItm, real_ot *MinCrd, int_ot ThrIdx )
  {
    OtrSct *otr       = (OtrSct *)OctIdx;
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];
    MshSct *msh       = otr->msh;
    VerSct TmpVer;
    TriSct *tri;
    int_ot i, EdgFlg       = 0;
    real_ot CurDis, MinDis = DBL_MAX;

    if ( typ == OTTypVer )
    {
      CpyVec( (real_ot *)GetPtrItm( msh, OTTypVer, MinItm ), MinCrd );
      return ( 1 );
    }
    else if ( typ == OTTypEdg )
    {
      SetItm( msh, OTTypEdg, MinItm, 0, ThrIdx );
      PrjVerLin( VerCrd, ThrMsh->edg.ver[ 0 ]->crd, ThrMsh->edg.tng, TmpVer.crd );

      if ( VerInsEdg( &ThrMsh->edg, &TmpVer, otr->eps ) )
      {
        CpyVec( TmpVer.crd, MinCrd );
        return ( 2 );
      }

      if ( dis( VerCrd, ThrMsh->edg.ver[ 0 ]->crd ) < dis( VerCrd, ThrMsh->edg.ver[ 1 ]->crd ) )
        CpyVec( ThrMsh->edg.ver[ 0 ]->crd, MinCrd );
      else
        CpyVec( ThrMsh->edg.ver[ 1 ]->crd, MinCrd );

      return ( 1 );
    }
    else if ( typ == OTTypTri )
    {
      SetItm( msh, OTTypTri, MinItm, TngFlg, ThrIdx );
      PrjVerPla( VerCrd, ThrMsh->tri.ver[ 0 ]->crd, ThrMsh->tri.nrm, TmpVer.crd );

      if ( VerInsTri( &ThrMsh->tri, &TmpVer, otr->eps ) )
      {
        CpyVec( TmpVer.crd, MinCrd );
        return ( 3 );
      }

      for ( i = 0; i < 3; i++ )
      {
        PrjVerLin( VerCrd, ThrMsh->tri.edg[ i ].ver[ 0 ]->crd, ThrMsh->tri.edg[ i ].tng, TmpVer.crd );

        if ( VerInsEdg( &ThrMsh->tri.edg[ i ], &TmpVer, otr->eps ) && ( dis( VerCrd, TmpVer.crd ) < MinDis ) )
        {
          MinDis = dis( VerCrd, TmpVer.crd );
          CpyVec( TmpVer.crd, MinCrd );
          EdgFlg = 2;
        }
      }

      for ( i = 0; i < 3; i++ )
      {
        CurDis = dis( VerCrd, ThrMsh->tri.ver[ i ]->crd );

        if ( CurDis < MinDis )
        {
          MinDis = CurDis;
          CpyVec( ThrMsh->tri.ver[ i ]->crd, MinCrd );
          EdgFlg = 0;
        }
      }

      if ( EdgFlg )
        return ( 2 );
      else
        return ( 1 );
    }
    else if ( typ == OTTypQad )
    {
      SetItm( msh, OTTypQad, MinItm, TngFlg, ThrIdx );

      for ( i = 0; i < 2; i++ )
      {
        tri = &ThrMsh->qad.tri[ i ];
        PrjVerPla( VerCrd, tri->ver[ 0 ]->crd, tri->nrm, TmpVer.crd );

        if ( VerInsTri( tri, &TmpVer, otr->eps ) )
        {
          CpyVec( TmpVer.crd, MinCrd );
          return ( 3 );
        }
      }

      for ( i = 0; i < 4; i++ )
      {
        PrjVerLin( VerCrd, ThrMsh->qad.edg[ i ].ver[ 0 ]->crd, ThrMsh->qad.edg[ i ].tng, TmpVer.crd );

        if ( VerInsEdg( &ThrMsh->qad.edg[ i ], &TmpVer, otr->eps ) && ( dis( VerCrd, TmpVer.crd ) < MinDis ) )
        {
          MinDis = dis( VerCrd, TmpVer.crd );
          CpyVec( TmpVer.crd, MinCrd );
          EdgFlg = 2;
        }
      }

      for ( i = 0; i < 4; i++ )
      {
        CurDis = dis( VerCrd, ThrMsh->qad.ver[ i ]->crd );

        if ( CurDis < MinDis )
        {
          MinDis = CurDis;
          CpyVec( ThrMsh->qad.ver[ i ]->crd, MinCrd );
          EdgFlg = 0;
        }
      }

      if ( EdgFlg )
        return ( 2 );
      else
        return ( 1 );
    }
    else
      return ( 0 );
  }

  static void GetBucBox( OtrSct *otr, BucSct *buc, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ )
    {
      MinCrd[ i ] = otr->bnd[ 0 ][ i ] + buc->pos[ i ] * otr->BucSiz;
      MaxCrd[ i ] = otr->bnd[ 0 ][ i ] + ( buc->pos[ i ] + 1 ) * otr->BucSiz;
    }
  }

  static BucSct *GetBucNgb( OtrSct *otr, BucSct *buc, int_ot dir )
  {
    if ( ( dir == 0 ) && ( buc->pos[ 0 ] > 0 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ] - 1, buc->pos[ 1 ], buc->pos[ 2 ], otr->GrdLvl ) ] );

    if ( ( dir == 1 ) && ( buc->pos[ 0 ] < otr->NmbBuc - 1 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ] + 1, buc->pos[ 1 ], buc->pos[ 2 ], otr->GrdLvl ) ] );

    if ( ( dir == 2 ) && ( buc->pos[ 1 ] > 0 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ], buc->pos[ 1 ] - 1, buc->pos[ 2 ], otr->GrdLvl ) ] );

    if ( ( dir == 3 ) && ( buc->pos[ 1 ] < otr->NmbBuc - 1 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ], buc->pos[ 1 ] + 1, buc->pos[ 2 ], otr->GrdLvl ) ] );

    if ( ( dir == 4 ) && ( buc->pos[ 2 ] > 0 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ], buc->pos[ 1 ], buc->pos[ 2 ] - 1, otr->GrdLvl ) ] );

    if ( ( dir == 5 ) && ( buc->pos[ 2 ] < otr->NmbBuc - 1 ) )
      return ( &otr->grd[ BUC( buc->pos[ 0 ], buc->pos[ 1 ], buc->pos[ 2 ] + 1, otr->GrdLvl ) ] );

    return ( NULL );
  }

  static real_ot DisVerOct( real_ot VerCrd[ 3 ], real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot ClpCrd[ 3 ];

    for ( i = 0; i < 3; i++ )
    {
      ClpCrd[ i ] = MAX( VerCrd[ i ], MinCrd[ i ] );
      ClpCrd[ i ] = MIN( ClpCrd[ i ], MaxCrd[ i ] );
    }

    return ( DisPow( ClpCrd, VerCrd ) );
  }

  static void GetOctLnk( MshSct *msh, int_ot typ, real_ot VerCrd[ 3 ], int_ot *MinItm, real_ot *MinDis, OctSct *oct,
                         real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ], int_ot( UsrPrc )( void *, int_ot ), void *UsrDat,
                         int_ot ThrIdx )
  {
    int_ot i;
    real_ot CurDis, SonMin[ 3 ], SonMax[ 3 ];
    LnkSct *lnk;
    MshThrSct *ThrMsh = msh->thr[ ThrIdx ];
#ifdef WITH_FAST_MODE
    OctSct *son;
#endif

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
#ifdef WITH_FAST_MODE
        son = &oct->son[ i ];

        if ( DisVerOct( VerCrd, son->MinCrd, son->MaxCrd ) <= *MinDis )
          GetOctLnk( msh, typ, VerCrd, MinItm, MinDis, son, son->MinCrd, son->MaxCrd, UsrPrc, UsrDat, ThrIdx );
#else
      SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );

      if ( DisVerOct( VerCrd, SonMin, SonMax ) <= *MinDis )
        GetOctLnk( msh, typ, VerCrd, MinItm, MinDis, oct->son + i, SonMin, SonMax, UsrPrc, UsrDat, ThrIdx );
#endif
      }
    }
    else if ( ( lnk = oct->lnk ) )
    {
      do
      {
        if ( lnk->typ != typ ) continue;

        if ( UsrPrc && !UsrPrc( UsrDat, lnk->idx ) ) continue;

        if ( lnk->typ == OTTypVer )
        {
          CurDis = DisPow( VerCrd, (real_ot *)GetPtrItm( msh, OTTypVer, lnk->idx ) );
        }
        else if ( lnk->typ == OTTypEdg )
        {
          SetItm( msh, OTTypEdg, lnk->idx, 0, ThrIdx );
          CurDis = DisVerEdg( VerCrd, &ThrMsh->edg );
        }
        else if ( lnk->typ == OTTypTri )
        {
          if ( ThrMsh->TagTab[ lnk->idx ] == ThrMsh->tag )
            continue;
          else
            ThrMsh->TagTab[ lnk->idx ] = ThrMsh->tag;

#ifdef WITH_FAST_MODE

          CurDis = DisVerTriStl( msh, VerCrd, &msh->StlTab[ lnk->idx ] );
#else

        SetItm( msh, OTTypTri, lnk->idx, 0, ThrIdx );
        CurDis = DisVerTri( msh, VerCrd, &ThrMsh->tri );
#endif
        }
        else if ( lnk->typ == OTTypQad )
        {
          if ( ThrMsh->TagTab[ lnk->idx ] == ThrMsh->tag )
            continue;
          else
            ThrMsh->TagTab[ lnk->idx ] = ThrMsh->tag;

          SetItm( msh, OTTypQad, lnk->idx, 0, ThrIdx );
          CurDis = DisVerQad( msh, VerCrd, &ThrMsh->qad );
        }

        if ( ( CurDis >= 0. ) && ( CurDis < *MinDis ) )
        {
          *MinItm = lnk->idx;
          *MinDis = CurDis;
        }
      } while ( ( lnk = lnk->nex ) );
    }
  }

#ifdef WITH_FAST_MODE
  static void CpyOctCrd( OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];

    CpyVec( MinCrd, oct->MinCrd );
    CpyVec( MaxCrd, oct->MaxCrd );

    if ( !oct->sub ) return;

    for ( i = 0; i < 8; i++ )
    {
      SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
      CpyOctCrd( oct->son + i, SonMin, SonMax );
    }
  }
#endif

  static void IntLinOct( OtrSct *otr, MshSct *msh, real_ot *crd, real_ot *tng, int_ot *MinItm, real_ot *MinDis,
                         OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ], int_ot( UsrPrc )( void *, int_ot ),
                         void *UsrDat, int_ot ThrIdx )
  {
    int_ot i;
    real_ot CurDis, SonMin[ 3 ], SonMax[ 3 ];
    VerSct IntVer;
    LnkSct *lnk;
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );

        if ( !LinIntBox( crd, tng, SonMin, SonMax, msh->eps ) ) continue;

        IntLinOct( otr, msh, crd, tng, MinItm, MinDis, oct->son + i, SonMin, SonMax, UsrPrc, UsrDat, ThrIdx );
      }
    }
    else if ( ( lnk = oct->lnk ) )
    {
      do
      {
        if ( lnk->typ != OTTypTri ) continue;

        if ( ThrMsh->TagTab[ lnk->idx ] == ThrMsh->tag ) continue;

        ThrMsh->TagTab[ lnk->idx ] = ThrMsh->tag;
        SetItm( msh, OTTypTri, lnk->idx, 0, ThrIdx );

        if ( UsrPrc && !UsrPrc( UsrDat, lnk->idx ) ) continue;

        if ( DotPrd( tng, ThrMsh->tri.nrm ) != 0. )
        {
          LinIntPla( crd, tng, ThrMsh->tri.ver[ 0 ]->crd, ThrMsh->tri.nrm, IntVer.crd );

          if ( VerInsTri( &ThrMsh->tri, &IntVer, msh->eps ) )
          {
            CurDis = DisPow( IntVer.crd, crd );

            if ( CurDis < *MinDis )
            {
              *MinItm = lnk->idx;
              *MinDis = CurDis;
            }
          }
        }
      } while ( ( lnk = lnk->nex ) );
    }
  }

  static void IntVecOct( OtrSct *otr, MshSct *msh, real_ot *crd, real_ot *tng, int_ot *NmbTri, int_ot *TriTab,
                         OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ], int_ot ThrIdx )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ], vec[ 3 ];
    VerSct IntVer;
    LnkSct *lnk;
    MshThrSct *ThrMsh = otr->msh->thr[ ThrIdx ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );

        if ( !LinIntBox( crd, tng, SonMin, SonMax, msh->eps ) ) continue;

        IntVecOct( otr, msh, crd, tng, NmbTri, TriTab, oct->son + i, SonMin, SonMax, ThrIdx );
      }
    }
    else if ( ( lnk = oct->lnk ) )
    {
      do
      {
        if ( lnk->typ != OTTypTri ) continue;

        if ( ThrMsh->TagTab[ lnk->idx ] == ThrMsh->tag ) continue;

        ThrMsh->TagTab[ lnk->idx ] = ThrMsh->tag;
        SetItm( msh, OTTypTri, lnk->idx, 0, ThrIdx );

        if ( DotPrd( tng, ThrMsh->tri.nrm ) != 0. )
        {
          LinIntPla( crd, tng, ThrMsh->tri.ver[ 0 ]->crd, ThrMsh->tri.nrm, IntVer.crd );

          if ( !VerInsTri( &ThrMsh->tri, &IntVer, msh->eps ) ) continue;

          SubVec3( crd, IntVer.crd, vec );

          if ( ( *NmbTri < MaxRayTri ) && ( DotPrd( tng, vec ) > 0. ) ) TriTab[ ( *NmbTri )++ ] = lnk->idx;
        }
      } while ( ( lnk = lnk->nex ) );
    }
  }

  static void SetItm( MshSct *msh, int_ot typ, int_ot idx, int_ot flg, int_ot ThrIdx )
  {
    int_ot i, j, *IdxTab;
    const int_ot TetEdgFac[ 6 ][ 2 ] = { { 2, 3 }, { 1, 3 }, { 1, 2 }, { 0, 3 }, { 0, 2 }, { 0, 1 } };
    real_ot *CrdTab;
    MshThrSct *ThrMsh = msh->thr[ ThrIdx ];

    if ( typ == OTTypVer )
    {
      ThrMsh->ver[ 0 ].idx = idx;
      CpyVec( (real_ot *)GetPtrItm( msh, typ, idx ), ThrMsh->ver[ 0 ].crd );
    }
    else if ( typ == OTTypEdg )
    {
      ThrMsh->edg.idx = idx;
      IdxTab          = (int_ot *)GetPtrItm( msh, typ, idx );

      for ( i = 0; i < 2; i++ ) CpyVec( (real_ot *)GetPtrItm( msh, OTTypVer, IdxTab[ i ] ), ThrMsh->edg.ver[ i ]->crd );

      SetEdgTng( &ThrMsh->edg );
    }
    else if ( typ == OTTypTri )
    {
      ThrMsh->tri.idx = idx;

      if ( msh->StlMod )
      {
        CrdTab = (real_ot *)GetPtrItm( msh, typ, idx );

        for ( i = 0; i < 3; i++ ) CpyVec( &CrdTab[ i * 3 ], ThrMsh->tri.ver[ i ]->crd );
      }
      else
      {
        IdxTab = (int_ot *)GetPtrItm( msh, typ, idx );

        for ( i = 0; i < 3; i++ )
          CpyVec( (real_ot *)GetPtrItm( msh, OTTypVer, IdxTab[ i ] ), ThrMsh->tri.ver[ i ]->crd );
      }

      SetTriNrm( &ThrMsh->tri );

      if ( flg & TngFlg )
        for ( i = 0; i < 3; i++ ) SetEdgTng( &ThrMsh->tri.edg[ i ] );

      if ( flg & AniFlg ) ThrMsh->tri.ani = (float)GetTriAni( &ThrMsh->tri );
    }
    else if ( typ == OTTypQad )
    {
      ThrMsh->qad.idx = idx;
      IdxTab          = (int_ot *)GetPtrItm( msh, typ, idx );

      for ( i = 0; i < 4; i++ ) CpyVec( (real_ot *)GetPtrItm( msh, OTTypVer, IdxTab[ i ] ), ThrMsh->qad.ver[ i ]->crd );

      if ( flg & TngFlg )
        for ( i = 0; i < 4; i++ ) SetEdgTng( &ThrMsh->qad.edg[ i ] );

      for ( i = 0; i < 2; i++ )
      {
        SetTriNrm( &ThrMsh->qad.tri[ i ] );

        if ( flg & TngFlg )
          for ( j = 0; j < 3; j++ ) SetEdgTng( &ThrMsh->qad.tri[ i ].edg[ j ] );

        if ( flg & AniFlg ) ThrMsh->tri.ani = (float)GetTriAni( &ThrMsh->qad.tri[ i ] );
      }
    }
  }

  static void BakMshItm( MshSct *msh )
  {
    memcpy( &msh->thr[ 0 ]->BakEdg, &msh->thr[ 0 ]->edg, sizeof( EdgSct ) );
    memcpy( &msh->thr[ 0 ]->BakTri, &msh->thr[ 0 ]->tri, sizeof( TriSct ) );
    memcpy( &msh->thr[ 0 ]->BakQad, &msh->thr[ 0 ]->qad, sizeof( QadSct ) );

    memcpy( msh->thr[ 0 ]->BakVer, msh->thr[ 0 ]->ver, 4 * sizeof( VerSct ) );
  }

  static void RstMshItm( MshSct *msh )
  {
    memcpy( &msh->thr[ 0 ]->edg, &msh->thr[ 0 ]->BakEdg, sizeof( EdgSct ) );
    memcpy( &msh->thr[ 0 ]->tri, &msh->thr[ 0 ]->BakTri, sizeof( TriSct ) );
    memcpy( &msh->thr[ 0 ]->qad, &msh->thr[ 0 ]->BakQad, sizeof( QadSct ) );

    memcpy( msh->thr[ 0 ]->ver, msh->thr[ 0 ]->BakVer, 4 * sizeof( VerSct ) );
  }

  static char *GetPtrItm( MshSct *msh, int_ot typ, int_ot idx )
  {
    return ( msh->UsrPtr[ typ ] + ( idx - msh->BasIdx ) * msh->UsrSiz[ typ ] );
  }

  static void SetMshBox( OtrSct *box, MshSct *msh )
  {
    int_ot i, j, k;
    real_ot MinCrd[ 3 ], MaxCrd[ 3 ], MidCrd[ 3 ], *CrdTab, siz;

    if ( msh->StlMod )
    {
      CpyVec( (real_ot *)GetPtrItm( msh, OTTypTri, msh->BasIdx ), MinCrd );
      CpyVec( MinCrd, MaxCrd );

      for ( i = 1; i < msh->NmbItm[ OTTypTri ]; i++ )
      {
        CrdTab = (real_ot *)GetPtrItm( msh, OTTypTri, i + msh->BasIdx );

        for ( j = 0; j < 3; j++ )
        {
          for ( k = 0; k < 3; k++ )
          {
            MinCrd[ k ] = MIN( MinCrd[ k ], CrdTab[ j * 3 + k ] );
            MaxCrd[ k ] = MAX( MaxCrd[ k ], CrdTab[ j * 3 + k ] );
          }
        }
      }
    }
    else
    {
      CpyVec( (real_ot *)GetPtrItm( msh, OTTypVer, msh->BasIdx ), MinCrd );
      CpyVec( MinCrd, MaxCrd );

      for ( i = 1; i < msh->NmbItm[ OTTypVer ]; i++ )
      {
        CrdTab = (real_ot *)GetPtrItm( msh, OTTypVer, i + msh->BasIdx );

        for ( j = 0; j < 3; j++ )
        {
          MinCrd[ j ] = MIN( MinCrd[ j ], CrdTab[ j ] );
          MaxCrd[ j ] = MAX( MaxCrd[ j ], CrdTab[ j ] );
        }
      }
    }

    siz         = MAX( MaxCrd[ 0 ] - MinCrd[ 0 ], MaxCrd[ 1 ] - MinCrd[ 1 ] );
    siz         = MAX( siz, MaxCrd[ 2 ] - MinCrd[ 2 ] );
    box->eps    = siz * FLT_EPSILON;
    box->MinSiz = box->MaxSiz = siz;

    LinCmbVec3( .5, MinCrd, .5, MaxCrd, MidCrd );
    AddScaVec1( siz * FLT_EPSILON, MidCrd );
    AddScaVec2( -1.02 * siz / 2., MidCrd, box->bnd[ 0 ] );
    AddScaVec2( 1.02 * siz / 2., MidCrd, box->bnd[ 1 ] );
  }

  static void AddVer( MshSct *msh, OtrSct *otr, OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );

        if ( VerInsOct( msh->thr[ 0 ]->ver[ 0 ].crd, SonMin, SonMax ) )
          AddVer( msh, otr, oct->son + i, SonMin, SonMax );
      }
    }
    else
    {
      LnkItm( otr, oct, OTTypVer, msh->thr[ 0 ]->ver[ 0 ].idx, 0 );

      if ( ( oct->lvl < otr->GrdLvl ) || ( ( oct->NmbVer >= oct->MaxItm ) && ( oct->lvl < MaxOctLvl ) ) )
      {
        SubOct( msh, otr, oct, MinCrd, MaxCrd );
      }
    }
  }

  static void AddEdg( MshSct *msh, OtrSct *otr, OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
        SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

        if ( EdgIntHex( &msh->thr[ 0 ]->edg, &otr->thr[ 0 ]->hex, otr->eps ) )
          AddEdg( msh, otr, oct->son + i, SonMin, SonMax );
      }
    }
    else
    {
      LnkItm( otr, oct, OTTypEdg, msh->thr[ 0 ]->edg.idx, 0 );

      if ( ( oct->lvl < otr->GrdLvl ) || ( ( oct->NmbEdg >= oct->MaxItm ) && ( oct->lvl < MaxOctLvl ) ) )
      {
        SubOct( msh, otr, oct, MinCrd, MaxCrd );
      }
    }
  }

  static void AddTri( MshSct *msh, OtrSct *otr, OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
        SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

        if ( TriIntHex( &msh->thr[ 0 ]->tri, &otr->thr[ 0 ]->hex, otr->eps ) )
          AddTri( msh, otr, oct->son + i, SonMin, SonMax );
      }
    }
    else
    {
      LnkItm( otr, oct, OTTypTri, msh->thr[ 0 ]->tri.idx, (unsigned char)msh->thr[ 0 ]->tri.ani );

      if ( ( oct->lvl < otr->GrdLvl ) || ( ( oct->NmbFac >= oct->MaxItm ) && ( oct->lvl < MaxOctLvl ) ) )
      {
        SubOct( msh, otr, oct, MinCrd, MaxCrd );
      }
    }
  }

  static void AddQad( MshSct *msh, OtrSct *otr, OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];

    if ( oct->sub )
    {
      for ( i = 0; i < 8; i++ )
      {
        SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
        SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

        if ( QadIntHex( &msh->thr[ 0 ]->qad, &otr->thr[ 0 ]->hex, otr->eps ) )
          AddQad( msh, otr, oct->son + i, SonMin, SonMax );
      }
    }
    else
    {
      LnkItm( otr, oct, OTTypQad, msh->thr[ 0 ]->qad.idx, 0 );

      if ( ( oct->lvl < otr->GrdLvl ) || ( ( oct->NmbFac >= oct->MaxItm ) && ( oct->lvl < MaxOctLvl ) ) )
      {
        SubOct( msh, otr, oct, MinCrd, MaxCrd );
      }
    }
  }

  static void SubOct( MshSct *msh, OtrSct *otr, OctSct *oct, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;
    real_ot SonMin[ 3 ], SonMax[ 3 ];
    LnkSct *lnk, *OctLnk = oct->lnk;
    OctSct *son;

    if ( !otr->NmbFreOct )
    {
      otr->CurOctBlk = (OctSct *)NewMem( otr, MemBlkSiz * 8 * sizeof( OctSct ) );
      otr->NmbFreOct = MemBlkSiz;
    }

    oct->son = &otr->CurOctBlk[ ( MemBlkSiz - otr->NmbFreOct-- ) * 8 ];
    oct->sub = 1;
    otr->NmbOct += 8;

    for ( i = 0; i < 8; i++ )
    {
      son         = oct->son + i;
      son->lnk    = NULL;
      son->son    = NULL;
      son->NmbVer = son->NmbEdg = son->NmbFac = son->NmbVol = 0;
      son->ani                                              = 1;
      son->MaxItm                                           = MaxItmOct;
      son->sub                                              = 0;
      son->lvl                                              = oct->lvl + 1;
    }

    otr->MinSiz = MIN( otr->MinSiz, ( MaxCrd[ 0 ] - MinCrd[ 0 ] ) / 2. );
    otr->MaxLvl = MAX( otr->MaxLvl, oct->lvl + 1 );

    BakMshItm( msh );

    while ( ( lnk = OctLnk ) )
    {
      if ( lnk->typ == OTTypVer )
      {
        SetItm( msh, OTTypVer, lnk->idx, 0, 0 );

        for ( i = 0; i < 8; i++ )
        {
          SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );

          if ( VerInsOct( msh->thr[ 0 ]->ver[ 0 ].crd, SonMin, SonMax ) )
            LnkItm( otr, oct->son + i, OTTypVer, lnk->idx, 0 );
        }
      }
      else if ( lnk->typ == OTTypEdg )
      {
        SetItm( msh, OTTypEdg, lnk->idx, 0, 0 );

        for ( i = 0; i < 8; i++ )
        {
          SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
          SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

          if ( EdgIntHex( &msh->thr[ 0 ]->edg, &otr->thr[ 0 ]->hex, otr->eps ) )
            LnkItm( otr, oct->son + i, OTTypEdg, lnk->idx, 0 );
        }
      }
      else if ( lnk->typ == OTTypTri )
      {
        SetItm( msh, OTTypTri, lnk->idx, TngFlg | AniFlg, 0 );

        for ( i = 0; i < 8; i++ )
        {
          SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
          SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

          if ( TriIntHex( &msh->thr[ 0 ]->tri, &otr->thr[ 0 ]->hex, otr->eps ) )
            LnkItm( otr, oct->son + i, OTTypTri, lnk->idx, oct->ani );
        }
      }
      else if ( lnk->typ == OTTypQad )
      {
        SetItm( msh, OTTypQad, lnk->idx, TngFlg | AniFlg, 0 );

        for ( i = 0; i < 8; i++ )
        {
          SetSonCrd( i, SonMin, SonMax, MinCrd, MaxCrd );
          SetTmpHex( &otr->thr[ 0 ]->hex, SonMin, SonMax );

          if ( QadIntHex( &msh->thr[ 0 ]->qad, &otr->thr[ 0 ]->hex, otr->eps ) )
            LnkItm( otr, oct->son + i, OTTypQad, lnk->idx, oct->ani );
        }
      }

      OctLnk         = lnk->nex;
      lnk->nex       = otr->NexFreLnk;
      otr->NexFreLnk = lnk;
    }

    RstMshItm( msh );
  }

  static void LnkItm( OtrSct *otr, OctSct *oct, int_ot typ, int_ot idx, unsigned char ani )
  {
    int_ot i;
    LnkSct *lnk = oct->lnk;

    while ( lnk )
    {
      if ( lnk->typ == typ && lnk->idx == idx ) return;

      lnk = lnk->nex;
    }

    if ( !otr->NexFreLnk )
    {
      otr->NexFreLnk = (LnkSct *)NewMem( otr, MemBlkSiz * sizeof( LnkSct ) );

      for ( i = 0; i < MemBlkSiz; i++ ) otr->NexFreLnk[ i ].nex = &otr->NexFreLnk[ i + 1 ];

      otr->NexFreLnk[ MemBlkSiz - 1 ].nex = NULL;
    }

    lnk            = otr->NexFreLnk;
    otr->NexFreLnk = lnk->nex;
    lnk->typ       = typ;
    lnk->idx       = idx;
    lnk->nex       = oct->lnk;
    oct->lnk       = lnk;

    if ( typ == OTTypVer )
      oct->NmbVer++;
    else if ( typ == OTTypEdg )
      oct->NmbEdg++;
    else if ( ( typ == OTTypTri ) || ( typ == OTTypQad ) )
    {
      if ( typ == OTTypTri )
      {
        oct->ani    = MIN( 255, ( oct->ani * oct->NmbFac + ani ) / ( oct->NmbFac + 1 ) );
        oct->MaxItm = MIN( MaxItmOct * ani, 255 );
      }

      oct->NmbFac++;
    }
    else
      oct->NmbVol++;
  }

  static void SetSonCrd( int_ot SonIdx, real_ot SonMin[ 3 ], real_ot SonMax[ 3 ], real_ot MinCrd[ 3 ],
                         real_ot MaxCrd[ 3 ] )
  {
    real_ot MidCrd[ 3 ];

    LinCmbVec3( .5, MinCrd, .5, MaxCrd, MidCrd );

    switch ( SonIdx )
    {
      case 0:
      {
        SonMin[ 0 ] = MinCrd[ 0 ];
        SonMin[ 1 ] = MinCrd[ 1 ];
        SonMin[ 2 ] = MinCrd[ 2 ];
        SonMax[ 0 ] = MidCrd[ 0 ];
        SonMax[ 1 ] = MidCrd[ 1 ];
        SonMax[ 2 ] = MidCrd[ 2 ];
      }
        return;

      case 1:
      {
        SonMin[ 0 ] = MidCrd[ 0 ];
        SonMin[ 1 ] = MinCrd[ 1 ];
        SonMin[ 2 ] = MinCrd[ 2 ];
        SonMax[ 0 ] = MaxCrd[ 0 ];
        SonMax[ 1 ] = MidCrd[ 1 ];
        SonMax[ 2 ] = MidCrd[ 2 ];
      }
        return;

      case 2:
      {
        SonMin[ 0 ] = MinCrd[ 0 ];
        SonMin[ 1 ] = MidCrd[ 1 ];
        SonMin[ 2 ] = MinCrd[ 2 ];
        SonMax[ 0 ] = MidCrd[ 0 ];
        SonMax[ 1 ] = MaxCrd[ 1 ];
        SonMax[ 2 ] = MidCrd[ 2 ];
      }
        return;

      case 3:
      {
        SonMin[ 0 ] = MidCrd[ 0 ];
        SonMin[ 1 ] = MidCrd[ 1 ];
        SonMin[ 2 ] = MinCrd[ 2 ];
        SonMax[ 0 ] = MaxCrd[ 0 ];
        SonMax[ 1 ] = MaxCrd[ 1 ];
        SonMax[ 2 ] = MidCrd[ 2 ];
      }
        return;

      case 4:
      {
        SonMin[ 0 ] = MinCrd[ 0 ];
        SonMin[ 1 ] = MinCrd[ 1 ];
        SonMin[ 2 ] = MidCrd[ 2 ];
        SonMax[ 0 ] = MidCrd[ 0 ];
        SonMax[ 1 ] = MidCrd[ 1 ];
        SonMax[ 2 ] = MaxCrd[ 2 ];
      }
        return;

      case 5:
      {
        SonMin[ 0 ] = MidCrd[ 0 ];
        SonMin[ 1 ] = MinCrd[ 1 ];
        SonMin[ 2 ] = MidCrd[ 2 ];
        SonMax[ 0 ] = MaxCrd[ 0 ];
        SonMax[ 1 ] = MidCrd[ 1 ];
        SonMax[ 2 ] = MaxCrd[ 2 ];
      }
        return;

      case 6:
      {
        SonMin[ 0 ] = MinCrd[ 0 ];
        SonMin[ 1 ] = MidCrd[ 1 ];
        SonMin[ 2 ] = MidCrd[ 2 ];
        SonMax[ 0 ] = MidCrd[ 0 ];
        SonMax[ 1 ] = MaxCrd[ 1 ];
        SonMax[ 2 ] = MaxCrd[ 2 ];
      }
        return;

      case 7:
      {
        SonMin[ 0 ] = MidCrd[ 0 ];
        SonMin[ 1 ] = MidCrd[ 1 ];
        SonMin[ 2 ] = MidCrd[ 2 ];
        SonMax[ 0 ] = MaxCrd[ 0 ];
        SonMax[ 1 ] = MaxCrd[ 1 ];
        SonMax[ 2 ] = MaxCrd[ 2 ];
      }
        return;
    }
  }

  static int_ot VerInsOct( real_ot VerCrd[ 3 ], real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ )
      if ( ( VerCrd[ i ] > MaxCrd[ i ] ) || ( VerCrd[ i ] < MinCrd[ i ] ) ) return ( 0 );

    return ( 1 );
  }

  static void SetTmpHex( HexSct *hex, real_ot MinCrd[ 3 ], real_ot MaxCrd[ 3 ] )
  {
    hex->ver[ 0 ]->crd[ 0 ] = MinCrd[ 0 ];
    hex->ver[ 0 ]->crd[ 1 ] = MinCrd[ 1 ];
    hex->ver[ 0 ]->crd[ 2 ] = MaxCrd[ 2 ];
    hex->ver[ 1 ]->crd[ 0 ] = MaxCrd[ 0 ];
    hex->ver[ 1 ]->crd[ 1 ] = MinCrd[ 1 ];
    hex->ver[ 1 ]->crd[ 2 ] = MaxCrd[ 2 ];
    hex->ver[ 2 ]->crd[ 0 ] = MaxCrd[ 0 ];
    hex->ver[ 2 ]->crd[ 1 ] = MinCrd[ 1 ];
    hex->ver[ 2 ]->crd[ 2 ] = MinCrd[ 2 ];
    hex->ver[ 3 ]->crd[ 0 ] = MinCrd[ 0 ];
    hex->ver[ 3 ]->crd[ 1 ] = MinCrd[ 1 ];
    hex->ver[ 3 ]->crd[ 2 ] = MinCrd[ 2 ];
    hex->ver[ 4 ]->crd[ 0 ] = MinCrd[ 0 ];
    hex->ver[ 4 ]->crd[ 1 ] = MaxCrd[ 1 ];
    hex->ver[ 4 ]->crd[ 2 ] = MaxCrd[ 2 ];
    hex->ver[ 5 ]->crd[ 0 ] = MaxCrd[ 0 ];
    hex->ver[ 5 ]->crd[ 1 ] = MaxCrd[ 1 ];
    hex->ver[ 5 ]->crd[ 2 ] = MaxCrd[ 2 ];
    hex->ver[ 6 ]->crd[ 0 ] = MaxCrd[ 0 ];
    hex->ver[ 6 ]->crd[ 1 ] = MaxCrd[ 1 ];
    hex->ver[ 6 ]->crd[ 2 ] = MinCrd[ 2 ];
    hex->ver[ 7 ]->crd[ 0 ] = MinCrd[ 0 ];
    hex->ver[ 7 ]->crd[ 1 ] = MaxCrd[ 1 ];
    hex->ver[ 7 ]->crd[ 2 ] = MinCrd[ 2 ];
  }

  static int_ot LinIntBox( real_ot *LinCrd, real_ot *LinTng, real_ot *BoxMin, real_ot *BoxMax, real_ot eps )
  {
    int_ot i;
    real_ot IntCrd[ 3 ], NrmTab[ 3 ][ 3 ] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

    for ( i = 0; i < 3; i++ )
    {
      if ( LinTng[ i ] == 0. ) continue;

      LinIntPla( LinCrd, LinTng, BoxMin, NrmTab[ i ], IntCrd );

      if ( VerInsBox( IntCrd, BoxMin, BoxMax, eps ) ) return ( 1 );

      LinIntPla( LinCrd, LinTng, BoxMax, NrmTab[ i ], IntCrd );

      if ( VerInsBox( IntCrd, BoxMin, BoxMax, eps ) ) return ( 1 );
    }

    return ( 0 );
  }

  static void LinIntPla( real_ot *LinCrd, real_ot *LinTng, real_ot *PlaCrd, real_ot *PlaNrm, real_ot *IntCrd )
  {
    real_ot u[ 3 ];

    SubVec3( LinCrd, PlaCrd, u );
    LinCmbVec3( 1., LinCrd, -DotPrd( PlaNrm, u ) / DotPrd( PlaNrm, LinTng ), LinTng, IntCrd );
  }

  static real_ot VerInsBox( real_ot *VerCrd, real_ot *BoxMin, real_ot *BoxMax, real_ot eps )
  {
    if ( ( VerCrd[ 0 ] < BoxMin[ 0 ] - eps ) || ( VerCrd[ 1 ] < BoxMin[ 1 ] - eps ) ||
         ( VerCrd[ 2 ] < BoxMin[ 2 ] - eps ) || ( VerCrd[ 0 ] > BoxMax[ 0 ] + eps ) ||
         ( VerCrd[ 1 ] > BoxMax[ 1 ] + eps ) || ( VerCrd[ 2 ] > BoxMax[ 2 ] + eps ) )
    {
      return ( 0 );
    }

    return ( 1 );
  }

  static int_ot EdgIntHex( EdgSct *edg, HexSct *hex, real_ot eps )
  {
    int_ot i;
    VerSct IntVer;

    if ( VerInsHex( edg->ver[ 0 ], hex ) || VerInsHex( edg->ver[ 1 ], hex ) ) return ( 1 );

    for ( i = 0; i < 6; i++ )
      if ( EdgIntQad( hex, i, edg, &IntVer, eps ) ) return ( 1 );

    return ( 0 );
  }

  static int_ot TriIntHex( TriSct *tri, HexSct *hex, real_ot eps )
  {
    int_ot i, j, pos, neg;
    real_ot CurDis;
    VerSct IntVer;

    if ( ( ( tri->ver[ 0 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) &&
           ( tri->ver[ 1 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) &&
           ( tri->ver[ 2 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) ) ||
         ( ( tri->ver[ 0 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) &&
           ( tri->ver[ 1 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) &&
           ( tri->ver[ 2 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) ) ||
         ( ( tri->ver[ 0 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) &&
           ( tri->ver[ 1 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) &&
           ( tri->ver[ 2 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) ) ||
         ( ( tri->ver[ 0 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) &&
           ( tri->ver[ 1 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) &&
           ( tri->ver[ 2 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) ) ||
         ( ( tri->ver[ 0 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) &&
           ( tri->ver[ 1 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) &&
           ( tri->ver[ 2 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) ) ||
         ( ( tri->ver[ 0 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) &&
           ( tri->ver[ 1 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) &&
           ( tri->ver[ 2 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) ) )
    {
      return ( 0 );
    }

    for ( i = 0; i < 3; i++ )
      if ( VerInsHex( tri->ver[ i ], hex ) ) return ( 1 );

    pos = neg = 0;

    for ( i = 0; i < 8; i++ )
    {
      CurDis = DisVerPla( hex->ver[ i ]->crd, tri->ver[ 0 ]->crd, tri->nrm );

      if ( CurDis < -eps )
        neg = 1;
      else if ( CurDis > eps )
        pos = 1;
      else
        pos = neg = 1;
    }

    if ( !pos || !neg ) return ( 0 );

    for ( i = 0; i < 6; i++ )
      for ( j = 0; j < 3; j++ )
        if ( EdgIntQad( hex, i, &tri->edg[ j ], &IntVer, eps ) ) return ( 1 );

    for ( i = 0; i < 12; i++ )
      if ( EdgIntTri( tri, &hex->edg[ i ], &IntVer, eps ) ) return ( 1 );

    return ( 0 );
  }

  static int_ot QadIntHex( QadSct *qad, HexSct *hex, real_ot eps )
  {
    if ( TriIntHex( &qad->tri[ 0 ], hex, eps ) || TriIntHex( &qad->tri[ 1 ], hex, eps ) )
      return ( 1 );
    else
      return ( 0 );
  }

  static int_ot TetIntHex( TetSct *tet, HexSct *hex, real_ot eps )
  {
    int_ot i, j;
    VerSct IntVer;

    if ( ( ( tet->ver[ 0 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) &&
           ( tet->ver[ 1 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) &&
           ( tet->ver[ 2 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) &&
           ( tet->ver[ 3 ]->crd[ 0 ] < hex->ver[ 3 ]->crd[ 0 ] ) ) ||
         ( ( tet->ver[ 0 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) &&
           ( tet->ver[ 1 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) &&
           ( tet->ver[ 2 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) &&
           ( tet->ver[ 3 ]->crd[ 0 ] > hex->ver[ 5 ]->crd[ 0 ] ) ) ||
         ( ( tet->ver[ 0 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) &&
           ( tet->ver[ 1 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) &&
           ( tet->ver[ 2 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) &&
           ( tet->ver[ 3 ]->crd[ 1 ] < hex->ver[ 3 ]->crd[ 1 ] ) ) ||
         ( ( tet->ver[ 0 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) &&
           ( tet->ver[ 1 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) &&
           ( tet->ver[ 2 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) &&
           ( tet->ver[ 3 ]->crd[ 1 ] > hex->ver[ 5 ]->crd[ 1 ] ) ) ||
         ( ( tet->ver[ 0 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) &&
           ( tet->ver[ 1 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) &&
           ( tet->ver[ 2 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) &&
           ( tet->ver[ 3 ]->crd[ 2 ] < hex->ver[ 3 ]->crd[ 2 ] ) ) ||
         ( ( tet->ver[ 0 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) &&
           ( tet->ver[ 1 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) &&
           ( tet->ver[ 2 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) &&
           ( tet->ver[ 3 ]->crd[ 2 ] > hex->ver[ 5 ]->crd[ 2 ] ) ) )
    {
      return ( 0 );
    }

    for ( i = 0; i < 4; i++ )
      if ( VerInsHex( tet->ver[ i ], hex ) ) return ( 1 );

    for ( i = 0; i < 8; i++ )
      if ( VerInsTet( hex->ver[ i ], tet, eps ) ) return ( 1 );

    for ( i = 0; i < 6; i++ )
      for ( j = 0; j < 6; j++ )
        if ( EdgIntQad( hex, i, &tet->edg[ j ], &IntVer, eps ) ) return ( 1 );

    for ( i = 0; i < 12; i++ )
      for ( j = 0; j < 4; j++ )
        if ( EdgIntTri( &tet->tri[ j ], &hex->edg[ i ], &IntVer, eps ) ) return ( 1 );

    return ( 0 );
  }

  int_ot VerInsTet( VerSct *ver, TetSct *tet, real_ot eps )
  {
    const int_ot TetFac[ 4 ][ 3 ] = { { 3, 2, 1 }, { 0, 2, 3 }, { 3, 1, 0 }, { 0, 1, 2 } };
    int_ot i, j, ins = 1;
    TetSct SubTet;

    SubTet.ver[ 3 ] = ver;

    for ( i = 0; i < 4; i++ )
    {
      for ( j = 0; j < 3; j++ ) SubTet.ver[ j ] = tet->ver[ TetFac[ i ][ j ] ];

      if ( GetVolTet( &SubTet ) <= 0 )
      {
        ins = 0;
        break;
      }
    }

    if ( ins ) return ( 1 );

    for ( i = 0; i < 4; i++ )
      if ( VerInsTri( &tet->tri[ i ], ver, eps ) ) return ( 1 );

    return ( 0 );
  }

  static int_ot VerInsHexWithTol( VerSct *ver, HexSct *hex, float eps )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ )
      if ( ( ver->crd[ i ] > hex->ver[ 5 ]->crd[ i ] + eps ) || ( ver->crd[ i ] < hex->ver[ 3 ]->crd[ i ] - eps ) )
      {
        return ( 0 );
      }

    return ( 1 );
  }

  static int_ot VerInsHex( VerSct *ver, HexSct *hex ) { return VerInsHexWithTol( ver, hex, 0 ); }

  static int_ot EdgIntQad( HexSct *hex, int_ot FacIdx, EdgSct *edg, VerSct *IntVer, real_ot eps )
  {
    int_ot i, NmbVer = 0;
    real_ot sgn[ 2 ];
    VerSct *ver = NULL;
    EdgSct edg2;
    QadSct *qad = &hex->qad[ FacIdx ];

    for ( i = 0; i < 2; i++ )
    {
      sgn[ i ] = DisVerPla( edg->ver[ i ]->crd, qad->ver[ 0 ]->crd, qad->nrm );

      if ( fabs( sgn[ i ] ) < eps )
      {
        ver = edg->ver[ i ];
        NmbVer++;
      }
    }

    switch ( NmbVer )
    {
      case 0:
      {
        if ( sgn[ 0 ] * sgn[ 1 ] < 0 )
        {
          LinCmbVec3( fabs( sgn[ 0 ] ) / ( fabs( sgn[ 0 ] ) + fabs( sgn[ 1 ] ) ), edg->ver[ 1 ]->crd,
                      fabs( sgn[ 1 ] ) / ( fabs( sgn[ 0 ] ) + fabs( sgn[ 1 ] ) ), edg->ver[ 0 ]->crd, IntVer->crd );

          return ( VerInsHexWithTol( IntVer, hex, eps ) );
        }
      }
      break;

      case 1:
      {
        if ( VerInsHex( ver, hex ) )
        {
          CpyVec( ver->crd, IntVer->crd );
          return ( 1 );
        }
      }
      break;

      case 2:
      {
        for ( i = 0; i < 4; i++ )
        {
          edg2.ver[ 0 ] = qad->ver[ i ];
          edg2.ver[ 1 ] = qad->ver[ ( i + 1 ) % 4 ];
          SetEdgTng( &edg2 );

          if ( EdgIntEdg( edg, &edg2, IntVer, eps ) ) return ( 1 );
        }
      }
      break;
    }

    return ( 0 );
  }

  static int_ot EdgIntTri( TriSct *tri, EdgSct *edg, VerSct *IntVer, real_ot eps )
  {
    int_ot i, NmbVer = 0;
    real_ot sgn[ 2 ];
    VerSct *ver = NULL;
    EdgSct edg2;

    for ( i = 0; i < 2; i++ )
    {
      sgn[ i ] = DisVerPla( edg->ver[ i ]->crd, tri->ver[ 0 ]->crd, tri->nrm );

      if ( fabs( sgn[ i ] ) < eps )
      {
        ver = edg->ver[ i ];
        NmbVer++;
      }
    }

    switch ( NmbVer )
    {
      case 0:
      {
        if ( sgn[ 0 ] * sgn[ 1 ] < 0 )
        {
          LinCmbVec3( fabs( sgn[ 0 ] ) / ( fabs( sgn[ 0 ] ) + fabs( sgn[ 1 ] ) ), edg->ver[ 1 ]->crd,
                      fabs( sgn[ 1 ] ) / ( fabs( sgn[ 0 ] ) + fabs( sgn[ 1 ] ) ), edg->ver[ 0 ]->crd, IntVer->crd );

          return ( VerInsTri( tri, IntVer, eps ) );
        }
      }
      break;

      case 1:
      {
        if ( VerInsTri( tri, ver, eps ) )
        {
          CpyVec( ver->crd, IntVer->crd );
          return ( 1 );
        }
      }
      break;

      case 2:
      {
        for ( i = 0; i < 3; i++ )
        {
          edg2.ver[ 0 ] = tri->ver[ i ];
          edg2.ver[ 1 ] = tri->ver[ ( i + 1 ) % 3 ];
          SetEdgTng( &edg2 );

          if ( EdgIntEdg( edg, &edg2, IntVer, eps ) ) return ( 1 );
        }
      }
      break;
    }

    return ( 0 );
  }

  static int_ot VerInsTri( TriSct *tri, VerSct *ver, real_ot eps )
  {
    int_ot i, ins = 1;
    real_ot vec[ 3 ][ 3 ], nrm[ 3 ];
    VerSct img;
    EdgSct edg;

    if ( PrjVerPla( ver->crd, tri->ver[ 0 ]->crd, tri->nrm, img.crd ) > eps ) return ( 0 );

    for ( i = 0; i < 3; i++ ) SubVec3( tri->ver[ i ]->crd, img.crd, vec[ i ] );

    for ( i = 0; i < 3; i++ )
    {
      CrsPrd( vec[ ( i + 1 ) % 3 ], vec[ i ], nrm );

      if ( DotPrd( nrm, tri->nrm ) <= 0 )
      {
        ins = 0;
        break;
      }
    }

    if ( ins ) return ( 1 );

    for ( i = 0; i < 3; i++ )
    {
      edg.ver[ 0 ] = tri->ver[ i ];
      edg.ver[ 1 ] = tri->ver[ ( i + 1 ) % 3 ];
      SetEdgTng( &edg );

      if ( VerInsEdg( &edg, &img, eps ) ) return ( 1 );
    }

    return ( 0 );
  }

  static int_ot EdgIntEdg( EdgSct *edg1, EdgSct *edg2, VerSct *IntVer, real_ot eps )
  {
    int_ot i, NmbVer = 0;
    real_ot siz[ 2 ];
    VerSct img, *ver = NULL;

    for ( i = 0; i < 2; i++ )
    {
      PrjVerLin( edg2->ver[ i ]->crd, edg1->ver[ 0 ]->crd, edg1->tng, img.crd );
      siz[ i ] = dis( edg2->ver[ i ]->crd, img.crd );

      if ( siz[ i ] < eps ) NmbVer++;
    }

    if ( NmbVer < 2 )
    {
      LinCmbVec3( siz[ 0 ] / ( siz[ 0 ] + siz[ 1 ] ), edg2->ver[ 1 ]->crd, siz[ 1 ] / ( siz[ 0 ] + siz[ 1 ] ),
                  edg2->ver[ 0 ]->crd, IntVer->crd );

      return ( VerInsEdg( edg1, IntVer, eps ) );
    }
    else
    {
      ver = NULL;

      if ( VerInsEdg( edg2, edg1->ver[ 0 ], eps ) )
        ver = edg1->ver[ 0 ];
      else if ( VerInsEdg( edg2, edg1->ver[ 1 ], eps ) )
        ver = edg1->ver[ 1 ];
      else if ( VerInsEdg( edg1, edg2->ver[ 0 ], eps ) )
        ver = edg2->ver[ 0 ];
      else if ( VerInsEdg( edg1, edg2->ver[ 1 ], eps ) )
        ver = edg2->ver[ 1 ];

      if ( ver )
      {
        CpyVec( ver->crd, IntVer->crd );
        return ( 1 );
      }
    }

    return ( 0 );
  }

  static int_ot VerInsEdg( EdgSct *edg, VerSct *ver, real_ot eps )
  {
    int_ot i;
    real_ot u[ 3 ], v[ 3 ];
    VerSct img;

    PrjVerLin( ver->crd, edg->ver[ 0 ]->crd, edg->tng, img.crd );

    if ( DisPow( ver->crd, img.crd ) > POW( eps ) ) return ( 0 );

    SubVec3( img.crd, edg->ver[ 0 ]->crd, u );
    SubVec3( img.crd, edg->ver[ 1 ]->crd, v );

    if ( DotPrd( u, v ) < 0 ) return ( 1 );

    for ( i = 0; i < 2; i++ )
      if ( DisPow( img.crd, edg->ver[ i ]->crd ) < POW( eps ) ) return ( 1 );

    return ( 0 );
  }

  static real_ot DisVerTri( MshSct *msh, real_ot VerCrd[ 3 ], TriSct *tri )
  {
    // int_ot i, cod = 0, inc = 1;
    // real_ot dis1, TriSrf;
    // VerSct img;
    // EdgSct edg;
    // TriSct SubTri;

    // GetTriVec( tri, tri->nrm );

    // if ( ( TriSrf = GetNrmVec( tri->nrm ) ) ) MulVec1( 1. / TriSrf, tri->nrm );

    // SubTri.ver[ 2 ] = &img;
    // dis1            = PrjVerPla( VerCrd, tri->ver[ 0 ]->crd, tri->nrm, img.crd );

    // for ( i = 0; i < 3; i++ )
    // {
    //   SubTri.ver[ 0 ] = tri->ver[ ( i + 1 ) % 3 ];
    //   SubTri.ver[ 1 ] = tri->ver[ ( i + 2 ) % 3 ];

    //   GetTriVec( &SubTri, SubTri.nrm );
    //   if ( DotPrd( SubTri.nrm, tri->nrm ) < 0. ) cod |= inc;

    //   inc = inc << 1;
    // }

    // switch ( cod )
    // {
    //   case 0:
    //     return POW( dis1 );

    //   case 1:
    //   {
    //     edg.ver[ 0 ] = tri->ver[ 1 ];
    //     edg.ver[ 1 ] = tri->ver[ 2 ];
    //     SetEdgTng( &edg );
    //     return ( DisVerEdg( VerCrd, &edg ) );
    //   }

    //   case 2:
    //   {
    //     edg.ver[ 0 ] = tri->ver[ 2 ];
    //     edg.ver[ 1 ] = tri->ver[ 0 ];
    //     SetEdgTng( &edg );
    //     return ( DisVerEdg( VerCrd, &edg ) );
    //   }

    //   case 3:
    //     return ( DisPow( VerCrd, tri->ver[ 2 ]->crd ) );

    //   case 4:
    //   {
    //     edg.ver[ 0 ] = tri->ver[ 0 ];
    //     edg.ver[ 1 ] = tri->ver[ 1 ];
    //     SetEdgTng( &edg );
    //     return ( DisVerEdg( VerCrd, &edg ) );
    //   }

    //   case 5:
    //     return ( DisPow( VerCrd, tri->ver[ 1 ]->crd ) );

    //   default:
    //   case 6:
    //     return ( DisPow( VerCrd, tri->ver[ 0 ]->crd ) );
    // }
    int_ot i, cod = 0, inc = 1;
    real_ot dis1, TriSrf;
    VerSct img;
    EdgSct edg;
    TriSct SubTri;

#ifdef WITH_PROFILING
    msh->CptTri++;
#endif

    // Compute the triangle's normal and surface
    // and project the coordinates on its plane
    GetTriVec( tri, tri->nrm );

    if ( ( TriSrf = GetNrmVec( tri->nrm ) ) ) MulVec1( 1. / TriSrf, tri->nrm );

    SubTri.ver[ 2 ] = &img;
    dis1            = PrjVerPla( VerCrd, tri->ver[ 0 ]->crd, tri->nrm, img.crd );

    // Check the projection's position
    for ( i = 0; i < 3; i++ )
    {
      SubTri.ver[ 0 ] = tri->ver[ ( i + 1 ) % 3 ];
      SubTri.ver[ 1 ] = tri->ver[ ( i + 2 ) % 3 ];

      GetTriVec( &SubTri, SubTri.nrm );
      if ( DotPrd( SubTri.nrm, tri->nrm ) < 0. ) cod |= inc;

      inc = inc << 1;
    }

    // Compute the distance between an edge
    // or a vertex depending on the position code
    switch ( cod )
    {
      // In the triangle
      case 0:
        return POW( dis1 );
      // Facing edge 0 (1-2)
      case 1:
      {
        edg.ver[ 0 ] = tri->ver[ 1 ];
        edg.ver[ 1 ] = tri->ver[ 2 ];
        SetEdgTng( &edg );
        return ( DisVerEdg( VerCrd, &edg ) );
      }

      // Facing edge 1 (2-0)
      case 2:
      {
        edg.ver[ 0 ] = tri->ver[ 2 ];
        edg.ver[ 1 ] = tri->ver[ 0 ];
        SetEdgTng( &edg );
        return ( DisVerEdg( VerCrd, &edg ) );
      }

      // Facing vertex 2
      case 3:
        return ( DisPow( VerCrd, tri->ver[ 2 ]->crd ) );

      // Facing edge 2 (0-1)
      case 4:
      {
        edg.ver[ 0 ] = tri->ver[ 0 ];
        edg.ver[ 1 ] = tri->ver[ 1 ];
        SetEdgTng( &edg );
        return ( DisVerEdg( VerCrd, &edg ) );
      }

      // Facing vertex 1
      case 5:
        return ( DisPow( VerCrd, tri->ver[ 1 ]->crd ) );

      // Facing vertex 0
      default:
      case 6:
        return ( DisPow( VerCrd, tri->ver[ 0 ]->crd ) );
    }
  }

#ifdef WITH_FAST_MODE
  static real_ot DisVerTriStl( MshSct *msh, real_ot VerCrd[ 3 ], StlSct *stl )
  {
    int_ot i, *IdxTab, cod = 0;
    real_ot dis1, dis2, SubSrf, ImgCrd[ 3 ], u[ 3 ], v[ 3 ], w[ 3 ], n[ 3 ][ 3 ], TotSrf;

#ifdef WITH_PROFILING
    msh->CptTri++;
#endif

    if ( stl->srf == 0. ) return ( -1. );

    dis1 = PrjVerPla( VerCrd, stl->crd[ 0 ], stl->nrm, ImgCrd );

    SubVec3( stl->crd[ 0 ], ImgCrd, u );
    SubVec3( stl->crd[ 1 ], ImgCrd, v );
    SubVec3( stl->crd[ 2 ], ImgCrd, w );

    CrsPrd( w, v, n[ 0 ] );
    CrsPrd( u, w, n[ 1 ] );
    CrsPrd( v, u, n[ 2 ] );

    if ( DotPrd( n[ 0 ], stl->nrm ) < 0. ) cod |= 1;

    if ( DotPrd( n[ 1 ], stl->nrm ) < 0. ) cod |= 2;

    if ( DotPrd( n[ 2 ], stl->nrm ) < 0. ) cod |= 4;

    TotSrf = GetNrmVec( n[ 0 ] ) + GetNrmVec( n[ 1 ] ) + GetNrmVec( n[ 2 ] );

    if ( ( TotSrf - stl->srf ) < .00001 * ( TotSrf + stl->srf ) ) return ( POW( dis1 ) );

    switch ( cod )
    {
      case 0:
      case 1:
      {
        return ( DisVerEdgStl( VerCrd, stl->crd[ 1 ], stl->crd[ 2 ], stl->tng[ 0 ], stl->siz[ 0 ] ) );
      }

      case 2:
      {
        return ( DisVerEdgStl( VerCrd, stl->crd[ 2 ], stl->crd[ 0 ], stl->tng[ 1 ], stl->siz[ 1 ] ) );
      }

      case 3:
        return ( DisPow( VerCrd, stl->crd[ 2 ] ) );

      case 4:
      {
        return ( DisVerEdgStl( VerCrd, stl->crd[ 0 ], stl->crd[ 1 ], stl->tng[ 2 ], stl->siz[ 2 ] ) );
      }

      case 5:
        return ( DisPow( VerCrd, stl->crd[ 1 ] ) );

      default:
      case 6:
        return ( DisPow( VerCrd, stl->crd[ 0 ] ) );
    }
  }
#endif

  static real_ot DisVerQad( MshSct *msh, real_ot VerCrd[ 3 ], QadSct *qad )
  {
    return ( MIN( DisVerTri( msh, VerCrd, &qad->tri[ 0 ] ), DisVerTri( msh, VerCrd, &qad->tri[ 1 ] ) ) );
  }

  static real_ot DisVerTet( MshSct *msh, real_ot *VerCrd, TetSct *tet )
  {
    int_ot i;
    real_ot CurDis, MinDis = DBL_MAX;
    VerSct TmpVer;

    CpyVec( VerCrd, TmpVer.crd );

    if ( VerInsTet( &TmpVer, tet, msh->eps ) ) return ( 0. );

    for ( i = 0; i < 4; i++ )
    {
      CurDis = DisPow( VerCrd, tet->ver[ i ]->crd );

      if ( CurDis < MinDis ) MinDis = CurDis;
    }

    return ( MinDis );
  }

  static real_ot GetTriSrf( TriSct *tri )
  {
    real_ot nrm[ 3 ];

    GetTriVec( tri, nrm );
    return ( GetNrmVec( nrm ) / 2. );
  }

  static real_ot GetVolTet( TetSct *tet )
  {
    real_ot c[ 9 ];

    c[ 0 ] = tet->ver[ 1 ]->crd[ 0 ] - tet->ver[ 0 ]->crd[ 0 ];
    c[ 1 ] = tet->ver[ 2 ]->crd[ 0 ] - tet->ver[ 0 ]->crd[ 0 ];
    c[ 2 ] = tet->ver[ 3 ]->crd[ 0 ] - tet->ver[ 0 ]->crd[ 0 ];

    c[ 3 ] = tet->ver[ 1 ]->crd[ 1 ] - tet->ver[ 0 ]->crd[ 1 ];
    c[ 4 ] = tet->ver[ 2 ]->crd[ 1 ] - tet->ver[ 0 ]->crd[ 1 ];
    c[ 5 ] = tet->ver[ 3 ]->crd[ 1 ] - tet->ver[ 0 ]->crd[ 1 ];

    c[ 6 ] = tet->ver[ 1 ]->crd[ 2 ] - tet->ver[ 0 ]->crd[ 2 ];
    c[ 7 ] = tet->ver[ 2 ]->crd[ 2 ] - tet->ver[ 0 ]->crd[ 2 ];
    c[ 8 ] = tet->ver[ 3 ]->crd[ 2 ] - tet->ver[ 0 ]->crd[ 2 ];

    return ( c[ 0 ] * ( c[ 4 ] * c[ 8 ] - c[ 5 ] * c[ 7 ] ) + c[ 1 ] * ( c[ 5 ] * c[ 6 ] - c[ 3 ] * c[ 8 ] ) +
             c[ 2 ] * ( c[ 3 ] * c[ 7 ] - c[ 4 ] * c[ 6 ] ) );
  }

  static real_ot DisVerEdg( real_ot VerCrd[ 3 ], EdgSct *edg )
  {
    real_ot dis0, dis1, TotSiz = 0.;
    VerSct img;

    PrjVerLin( VerCrd, edg->ver[ 0 ]->crd, edg->tng, img.crd );
    TotSiz = dis( edg->ver[ 0 ]->crd, img.crd ) + dis( edg->ver[ 1 ]->crd, img.crd );

    if ( ( TotSiz - edg->siz ) < .00001 * ( TotSiz + edg->siz ) ) return ( DisPow( VerCrd, img.crd ) );

    dis0 = DisPow( VerCrd, edg->ver[ 0 ]->crd );
    dis1 = DisPow( VerCrd, edg->ver[ 1 ]->crd );

    return ( MIN( dis0, dis1 ) );
  }

#ifdef WITH_FAST_MODE
  static real_ot DisVerEdgStl( real_ot VerCrd[ 3 ], real_ot *crd0, real_ot *crd1, real_ot *tng, real_ot siz )
  {
    real_ot dis0, dis1, ImgCrd[ 3 ], TotSiz = 0.;

    PrjVerLin( VerCrd, crd0, tng, ImgCrd );
    TotSiz = dis( crd0, ImgCrd ) + dis( crd1, ImgCrd );

    if ( ( TotSiz - siz ) < .00001 * ( TotSiz + siz ) ) return ( DisPow( VerCrd, ImgCrd ) );

    dis0 = DisPow( VerCrd, crd0 );
    dis1 = DisPow( VerCrd, crd1 );

    return ( MIN( dis0, dis1 ) );
  }
#endif

  static void GetTriVec( TriSct *tri, real_ot w[ 3 ] )
  {
    real_ot u[ 3 ], v[ 3 ];

    SubVec3( tri->ver[ 1 ]->crd, tri->ver[ 0 ]->crd, u );
    SubVec3( tri->ver[ 2 ]->crd, tri->ver[ 0 ]->crd, v );
    CrsPrd( v, u, w );
  }

  static void SetTriNrm( TriSct *tri )
  {
    GetTriVec( tri, tri->nrm );
    tri->srf = GetNrmVec( tri->nrm );

#if __STDC_VERSION__ >= 199901L
    if ( fpclassify( tri->srf ) == FP_ZERO )
      ClrVec( tri->nrm );
    else
      MulVec1( 1. / tri->srf, tri->nrm );
#else
  if ( tri->srf == 0. )
    ClrVec( tri->nrm );
  else
    MulVec1( 1. / tri->srf, tri->nrm );
#endif
  }

  static void SetEdgTng( EdgSct *edg )
  {
    SubVec3( edg->ver[ 1 ]->crd, edg->ver[ 0 ]->crd, edg->tng );
    edg->siz = GetNrmVec( edg->tng );

    if ( edg->siz ) MulVec1( 1. / edg->siz, edg->tng );
  }

  static void PrjVerLin( real_ot VerCrd[ 3 ], real_ot LinCrd[ 3 ], real_ot LinTng[ 3 ], real_ot ImgCrd[ 3 ] )
  {
    real_ot dp, u[ 3 ];

    SubVec3( VerCrd, LinCrd, u );
    dp = DotPrd( u, LinTng );
    LinCmbVec3( 1., LinCrd, dp, LinTng, ImgCrd );
  }

  static real_ot PrjVerPla( real_ot VerCrd[ 3 ], real_ot PlaCrd[ 3 ], real_ot PlaNrm[ 3 ], real_ot ImgCrd[ 3 ] )
  {
    real_ot DisPla, u[ 3 ];

    SubVec3( PlaCrd, VerCrd, u );
    DisPla = DotPrd( u, PlaNrm );
    MulVec2( DisPla, PlaNrm, ImgCrd );
    AddVec2( VerCrd, ImgCrd );

    return ( fabs( DisPla ) );
  }

  static real_ot DisVerPla( real_ot VerCrd[ 3 ], real_ot PlaCrd[ 3 ], real_ot PlaNrm[ 3 ] )
  {
    real_ot vec[ 3 ];

    SubVec3( VerCrd, PlaCrd, vec );
    return ( DotPrd( vec, PlaNrm ) );
  }

  static int_ot BoxIntBox( real_ot box1[ 2 ][ 3 ], real_ot box2[ 2 ][ 3 ], real_ot eps )
  {
    if ( ( ( ( box1[ 0 ][ 0 ] > box2[ 0 ][ 0 ] - eps ) && ( box1[ 0 ][ 0 ] < box2[ 1 ][ 0 ] + eps ) ) ||
           ( ( box1[ 1 ][ 0 ] > box2[ 0 ][ 0 ] - eps ) && ( box1[ 1 ][ 0 ] < box2[ 1 ][ 0 ] + eps ) ) ||
           ( ( box1[ 0 ][ 0 ] < box2[ 0 ][ 0 ] ) && ( box1[ 1 ][ 0 ] > box2[ 1 ][ 0 ] ) ) ) &&
         ( ( ( box1[ 0 ][ 1 ] > box2[ 0 ][ 1 ] - eps ) && ( box1[ 0 ][ 1 ] < box2[ 1 ][ 1 ] + eps ) ) ||
           ( ( box1[ 1 ][ 1 ] > box2[ 0 ][ 1 ] - eps ) && ( box1[ 1 ][ 1 ] < box2[ 1 ][ 1 ] + eps ) ) ||
           ( ( box1[ 0 ][ 1 ] < box2[ 0 ][ 1 ] ) && ( box1[ 1 ][ 1 ] > box2[ 1 ][ 1 ] ) ) ) &&
         ( ( ( box1[ 0 ][ 2 ] > box2[ 0 ][ 2 ] - eps ) && ( box1[ 0 ][ 2 ] < box2[ 1 ][ 2 ] + eps ) ) ||
           ( ( box1[ 1 ][ 2 ] > box2[ 0 ][ 2 ] - eps ) && ( box1[ 1 ][ 2 ] < box2[ 1 ][ 2 ] + eps ) ) ||
           ( ( box1[ 0 ][ 2 ] < box2[ 0 ][ 2 ] ) && ( box1[ 1 ][ 2 ] > box2[ 1 ][ 2 ] ) ) ) )
    {
      return ( 1 );
    }

    return ( 0 );
  }

  static real_ot GetTriAni( TriSct *tri )
  {
    real_ot srf, len, MaxLen;

    srf    = GetTriSrf( tri );
    MaxLen = len = DisPow( tri->ver[ 0 ]->crd, tri->ver[ 1 ]->crd );
    len          = DisPow( tri->ver[ 1 ]->crd, tri->ver[ 2 ]->crd );
    MaxLen       = MAX( len, MaxLen );
    len          = DisPow( tri->ver[ 2 ]->crd, tri->ver[ 0 ]->crd );
    MaxLen       = MAX( len, MaxLen );

    if ( MaxLen > 255. * 255. * srf )
      return ( 255. );
    else
      return ( sqrt( MaxLen / srf ) );
  }

  static real_ot dis( real_ot a[ 3 ], real_ot b[ 3 ] )
  {
    int_ot i;
    real_ot siz = 0;

    for ( i = 0; i < 3; i++ ) siz += POW( b[ i ] - a[ i ] );

    return ( sqrt( siz ) );
  }

  static real_ot DisPow( real_ot a[ 3 ], real_ot b[ 3 ] )
  {
    int_ot i;
    real_ot siz = 0;

    for ( i = 0; i < 3; i++ ) siz += POW( b[ i ] - a[ i ] );

    return ( siz );
  }

  static void SubVec3( real_ot u[ 3 ], real_ot v[ 3 ], real_ot w[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) w[ i ] = u[ i ] - v[ i ];
  }

  static real_ot DotPrd( real_ot u[ 3 ], real_ot v[ 3 ] )
  {
    int_ot i;
    real_ot dp = 0;

    for ( i = 0; i < 3; i++ ) dp += u[ i ] * v[ i ];

    return ( dp );
  }

  static void CrsPrd( real_ot u[ 3 ], real_ot v[ 3 ], real_ot w[ 3 ] )
  {
    w[ 0 ] = u[ 1 ] * v[ 2 ] - u[ 2 ] * v[ 1 ];
    w[ 1 ] = u[ 2 ] * v[ 0 ] - u[ 0 ] * v[ 2 ];
    w[ 2 ] = u[ 0 ] * v[ 1 ] - u[ 1 ] * v[ 0 ];
  }

  static void LinCmbVec3( real_ot w1, real_ot v1[ 3 ], real_ot w2, real_ot v2[ 3 ], real_ot v3[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) v3[ i ] = w1 * v1[ i ] + w2 * v2[ i ];
  }

  static void ClrVec( real_ot u[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) u[ i ] = 0.;
  }

  static void CpyVec( real_ot u[ 3 ], real_ot v[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) v[ i ] = u[ i ];
  }

  static void AddVec2( real_ot u[ 3 ], real_ot v[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) v[ i ] += u[ i ];
  }

  static void AddScaVec1( real_ot s, real_ot u[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) u[ i ] += s;
  }

  static void AddScaVec2( real_ot s, real_ot u[ 3 ], real_ot v[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) v[ i ] = u[ i ] + s;
  }

  static void MulVec1( real_ot w, real_ot u[ 3 ] )
  {
    u[ 0 ] *= w;
    u[ 1 ] *= w;
    u[ 2 ] *= w;
  }

  static void MulVec2( real_ot w, real_ot u[ 3 ], real_ot v[ 3 ] )
  {
    int_ot i;

    for ( i = 0; i < 3; i++ ) v[ i ] = w * u[ i ];
  }

  static real_ot GetNrmVec( real_ot u[ 3 ] ) { return ( sqrt( POW( u[ 0 ] ) + POW( u[ 1 ] ) + POW( u[ 2 ] ) ) ); }

  static void *NewMem( OtrSct *otr, size_t siz )
  {
    MemSct *mem;

    mem = (MemSct *)malloc( sizeof( MemSct ) );
    assert( mem );
    mem->adr = malloc( siz );
    assert( mem->adr );
    mem->siz    = siz;
    mem->nex    = otr->NexMem;
    otr->NexMem = mem;
    otr->MemUse += siz;

    return ( mem->adr );
  }

  static void FreAllMem( OtrSct *otr )
  {
    MemSct *mem = otr->NexMem, *nex;

    while ( mem )
    {
      otr->MemUse -= mem->siz;
      nex = mem->nex;
      free( mem->adr );
      free( mem );
      mem = nex;
    }
  }

#ifdef __cplusplus
}
#endif
