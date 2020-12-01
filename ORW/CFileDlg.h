#pragma once
#include <Windows.h>

#include <string>

class CFileDlg
{
public:
	CFileDlg(const char* filter, const char* def_ext);
	~CFileDlg();

	std::string GetOpenFileName();
	std::string GetSaveFileName();

protected:
	OPENFILENAMEA ofn;
	char szBuff[MAX_PATH + 1];
};

