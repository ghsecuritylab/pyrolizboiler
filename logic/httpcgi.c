#include "httpcgi.h"
#include "lwip/apps/httpd.h"


static const char* cgi_handler(int iIndex,
                               int iNumParams,
                               char *pcParam[],
                               char *pcValue[])
{
    return "/index.html";
}

void initHttpCgiServer()
{
    tCGI cgih = {"/leds.cgi", cgi_handler};
    http_set_cgi_handlers(&cgih, 1);

    httpd_init();
}
