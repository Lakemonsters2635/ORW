#include "all_includes.h"

#include "CSettings.h"

CSettings g_settings;



CSettings::CSettings()
{
}

CSettings::~CSettings()
{
    SaveIfNeeded();
}


void CSettings::New()
{
    SaveIfNeeded();
    Clear("");
    m_Filename = "";
}

bool CSettings::Load(const std::string& filename)
{
    SaveIfNeeded();

    char szDir[MAX_PATH + 1];
    ::GetCurrentDirectoryA(MAX_PATH + 1, szDir);

    if (filename.empty())
    {
        OPENFILENAMEA ofn;                      // common dialog box structure
        char szFile[MAX_PATH + 1] = { 0 };

        // Initialize OPENFILENAME
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = MAX_PATH + 1;
        ofn.lpstrFilter = "ORW Settings\0*.orw\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = NULL;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
        ofn.lpstrDefExt = "orw";
        ofn.lpstrInitialDir = szDir;

        if (GetOpenFileNameA(&ofn) != TRUE)
            return false;                             // User cancelled

        m_Filename = ofn.lpstrFile;
    }
    else
    {
        m_Filename = filename;
    }

    Clear("");
    pt::read_xml(m_Filename, m_Tree);
    m_bModified = false;

    return true;
}

bool CSettings::Save()
{
#ifndef _WIN32
#error "This code needs to be ported to Linux"
#else
    char szDir[MAX_PATH + 1];
    ::GetCurrentDirectoryA(MAX_PATH + 1, szDir);

    if (m_Filename == "")
    {
        OPENFILENAMEA ofn;                      // common dialog box structure
        char szFile[MAX_PATH+1] = { 0 };

        // Initialize OPENFILENAME
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = MAX_PATH+1;
        ofn.lpstrFilter = "ORW Settings\0*.orw\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = NULL;
        ofn.Flags = OFN_OVERWRITEPROMPT;
        ofn.lpstrDefExt = "orw";
        ofn.lpstrInitialDir = szDir;

        if (GetSaveFileNameA(&ofn) != TRUE)
            return false;                             // User cancelled

        m_Filename = ofn.lpstrFile;
    }
#endif

    Save(m_Filename);
    return true;
}

void CSettings::SaveAs()
{
    std::string saveFilename = m_Filename;
    m_Filename = "";

    if (Save())
        return;

    m_Filename = saveFilename;
    return;
}

void CSettings::Save(const std::string& filename)
{
   m_bModified = false;

   pt::write_xml(filename, m_Tree, std::locale(),
       pt::xml_writer_make_settings<std::string>(' ', 4));
}

void CSettings::SaveIfNeeded()
{
    if (!m_bModified)
        return;

#ifndef _WIN32
#error "This code needs to be ported to Linux"
#else
    std::string filename = m_Filename.empty() ? "Untitled" : m_Filename;
    
    if (::MessageBoxA(NULL, (std::string("Do you want to save changes to ") + filename).c_str(), "Save?", MB_YESNO | MB_ICONQUESTION) == IDYES)
    {
        Save();
    }
#endif
}

void CSettings::Clear(const std::string& group)
{
    m_Tree.erase(group);
}

std::string CSettings::ReplaceSpaces(const std::string& str)
{
    std::string res = str;
    size_t i;

    while ((i = res.find(' ')) != std::string::npos)
    {
        res = res.substr(0, i) + "_" + res.substr(i+1);
    }

    while ((i = res.find('#')) != std::string::npos)
    {
        res = res.substr(0, i) + res.substr(i + 1);
    }

    return res;
}
