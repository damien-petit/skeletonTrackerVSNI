
Note: if you meet this error during the compilation:

In file included from /usr/include/ni/XnStatusCodes.h:28:0,
                 from /usr/include/ni/XnMacros.h:28,
                 from /usr/include/ni/XnOS.h:29,
                 from /home/fkeith/software/wip/devel-src/joven/skeletonTrackerVSNI/include/skeletonTrackerVSNI.h:22,
                 from /home/fkeith/software/wip/devel-src/joven/skeletonTrackerVSNI/src/skeletonTrackerVSNI.cpp:1:
/usr/include/ni/XnStatus.h:50:67: error: two or more data types in declaration of ‘parameter’
/usr/include/ni/XnStatus.h:59:65: error: two or more data types in declaration of ‘parameter’
/usr/include/ni/XnStatus.h:68:53: error: two or more data types in declaration of ‘parameter’
/usr/include/ni/XnStatus.h:68:53: error: variable or field ‘xnPrintError’ declared void
/usr/include/ni/XnStatus.h:68:38: error: expected primary-expression before ‘const’
/usr/include/ni/XnStatus.h:68:61: error: expected primary-expression before ‘const’

The solution consists in modifying the lines 50, 59, 68 of /usr/include/ni/XnStatus.h
and change "const XnStatus Status" into "const XnStatus status", as such:

XN_C_API const XnChar* XN_C_DECL xnGetStatusString(const XnStatus Status);
XN_C_API const XnChar* XN_C_DECL xnGetStatusName(const XnStatus Status);
XN_C_API void XN_C_DECL xnPrintError(const XnStatus status, const XnChar* csUserMessage);


