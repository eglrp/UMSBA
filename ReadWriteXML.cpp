#include "ReadWriteXML.h"

int readXML(char *xmlFile, char *handlerRef){

	try {
		XMLPlatformUtils::Initialize();
	}
	catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		cout << "Error during initialization! :\n";
		cout << "Exception message is: \n"
			<< message << "\n";
		XMLString::release(&message);
		return 1;
	}

	SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();
	parser->setFeature(XMLUni::fgSAX2CoreValidation, true);
	parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, true);   // optional


	if (strcmp(handlerRef, "icf") == 0){
		ICFHandler* defaultHandler = new ICFHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
		
	}

	if (strcmp(handlerRef, "prj") == 0){
		PRJHandler *defaultHandler = new PRJHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
		
	}

	if (strcmp(handlerRef, "cam") == 0){

		CAMHandler *defaultHandler = new CAMHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
	}

	if (strcmp(handlerRef, "gps") == 0){

		GPSHandler *defaultHandler = new GPSHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
	}

	if (strcmp(handlerRef, "gcp") == 0){

		GCPHandler *defaultHandler = new GCPHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
	}

	if (strcmp(handlerRef, "dis") == 0){

		DISHandler *defaultHandler = new DISHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
	}

	if (strcmp(handlerRef, "pcf") == 0){

		PCFHandler *defaultHandler = new PCFHandler();
		parser->setContentHandler(defaultHandler);
		parser->setErrorHandler(defaultHandler);
	}


	try {
		parser->parse(xmlFile);
	}
	catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		cout << "Exception message is: \n"
			<< message << "\n";
		XMLString::release(&message);
		return -1;
	}
	catch (const SAXParseException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		cout << "Exception message is: \n"
			<< message << "\n";
		XMLString::release(&message);
		return -1;
	}
	catch (...) {
		cout << "Unexpected Exception \n";
		return -1;
	}

	delete parser;
	//delete defaultHandler;
    return 0;
}
