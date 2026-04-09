Unicode True
SetCompressor /SOLID lzma
RequestExecutionLevel user
ShowInstDetails show
ShowUnInstDetails show

!include "MUI2.nsh"
!include "x64.nsh"
!include "FileFunc.nsh"

!ifndef APP_NAME
!define APP_NAME "Gpredict"
!endif

!ifndef APP_VERSION
!define APP_VERSION "dev"
!endif

!ifndef STAGE_DIR
!error "STAGE_DIR define is required"
!endif

!ifndef OUT_FILE
!define OUT_FILE "gpredict-setup.exe"
!endif

!define REPO_ROOT "${__FILEDIR__}\..\.."
!define APP_ICON "${REPO_ROOT}\win32\icons\gpredict-icon.ico"
!define LICENSE_FILE "${REPO_ROOT}\COPYING"

!define COMPANY_NAME "Alexandru Csete, OZ9AEC / Matteo Avigni"
!define PRODUCT_PUBLISHER "Alexandru Csete, OZ9AEC / Matteo Avigni"
!define PRODUCT_URL "https://community.libre.space/c/gpredict"

Name "${APP_NAME} ${APP_VERSION}"
OutFile "${OUT_FILE}"
BrandingText "${APP_NAME} ${APP_VERSION}"
InstallDir "$LocalAppData\Programs\Gpredict"
InstallDirRegKey HKCU "Software\Gpredict" "InstallDir"

!define MUI_ABORTWARNING
!define MUI_ICON "${APP_ICON}"
!define MUI_UNICON "${APP_ICON}"
!define MUI_FINISHPAGE_RUN_FUNCTION LaunchGpredict
!define MUI_FINISHPAGE_RUN_TEXT "Launch Gpredict"

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE "${LICENSE_FILE}"
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_WELCOME
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH

!insertmacro MUI_LANGUAGE "English"

Function LaunchGpredict
  ExecShell "" "$INSTDIR\gpredict.exe"
FunctionEnd

Section "Gpredict" SecMain
  SetShellVarContext current
  ${If} ${RunningX64}
    SetRegView 64
  ${EndIf}

  SetOutPath "$INSTDIR"
  File /r "${STAGE_DIR}\*"
  WriteUninstaller "$INSTDIR\Uninstall.exe"

  CreateDirectory "$SMPROGRAMS\Gpredict"
  CreateShortcut "$SMPROGRAMS\Gpredict\Gpredict.lnk" "$INSTDIR\gpredict.exe"
  CreateShortcut "$SMPROGRAMS\Gpredict\Uninstall Gpredict.lnk" "$INSTDIR\Uninstall.exe"

  WriteRegStr HKCU "Software\Gpredict" "InstallDir" "$INSTDIR"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "DisplayName" "${APP_NAME}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "DisplayVersion" "${APP_VERSION}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "Publisher" "${PRODUCT_PUBLISHER}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "InstallLocation" "$INSTDIR"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "DisplayIcon" "$INSTDIR\gpredict.exe"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "URLInfoAbout" "${PRODUCT_URL}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "UninstallString" "$INSTDIR\Uninstall.exe"
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "NoModify" 1
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict" "NoRepair" 1
SectionEnd

Section "Uninstall"
  SetShellVarContext current
  ${If} ${RunningX64}
    SetRegView 64
  ${EndIf}

  Delete "$SMPROGRAMS\Gpredict\Gpredict.lnk"
  Delete "$SMPROGRAMS\Gpredict\Uninstall Gpredict.lnk"
  RMDir "$SMPROGRAMS\Gpredict"

  DeleteRegKey HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\Gpredict"
  DeleteRegKey HKCU "Software\Gpredict"

  Delete "$INSTDIR\Uninstall.exe"
  RMDir /r "$INSTDIR"
SectionEnd
