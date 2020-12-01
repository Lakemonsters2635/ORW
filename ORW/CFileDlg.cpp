#include "all_includes.h"
#include "CFileDlg.h"

CFileDlg::CFileDlg(const char* filter, const char* def_ext)
{
#ifndef _WIN32
#error "This code needs to be ported to Linux"
#else
    char szDir[MAX_PATH + 1];
    ::GetCurrentDirectoryA(MAX_PATH + 1, szDir);

    // Initialize OPENFILENAME
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = NULL;
    ofn.lpstrFile = szBuff;
    szBuff[0] = 0;
    ofn.nMaxFile = sizeof(szBuff);

    // Filter has embedded NULs.  We need to get its actual length

    const char* eof = strlen(filter) + filter;
    while (eof[1])
        eof = strlen(eof + 1) + eof + 1;

    int len = (eof - filter) + 2;
    ofn.lpstrFilter = new char[len];

    memcpy((void*)ofn.lpstrFilter, filter, len);

    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = NULL;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.lpstrDefExt = _strdup(def_ext);
    ofn.lpstrInitialDir = szDir;
#endif
}

CFileDlg::~CFileDlg()
{
    delete ofn.lpstrFilter;
    delete ofn.lpstrDefExt;
}

std::string CFileDlg::GetOpenFileName()
{
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    if (GetOpenFileNameA(&ofn) != TRUE)
    {
        DWORD dwErr = CommDlgExtendedError();
        return "";                             // User cancelled
    }

    return ofn.lpstrFile;
}

std::string CFileDlg::GetSaveFileName()
{
    ofn.Flags = OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn) != TRUE)
        return "";                             // User cancelled

    return ofn.lpstrFile;
}
