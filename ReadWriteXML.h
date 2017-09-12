#include "main.h"
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include "CICFHandler.h"
#include "CPRJHandler.h"
#include "CCAMHandler.h"
#include "CGCPHandler.h"
#include "CGPSHandler.h"
#include "CDISHandler.h"
#include "CPCFHandler.h"

using namespace xercesc;

int readXML(const char *fileName, char *handlerRef);
