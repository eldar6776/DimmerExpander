Const ForReading = 1
Const ForWriting = 2

Set objFSO = CreateObject("Scripting.FileSystemObject")
Set objTextFile = objFSO.OpenTextFile("..\..\Common\fwbuild.h", ForReading)
strNewFwDate = strNewFwDate & objTextFile.ReadLine
strNewFwTime = strNewFwTime & objTextFile.ReadLine
objTextFile.Close

Set objTextFile = objFSO.OpenTextFile("..\..\Common\common.h", ForReading)
Do Until objTextFile.AtEndOfStream
    strLine = objTextFile.ReadLine
    intFailure = InStr(strLine, "FW_DATE")
    If intFailure > 0 And intFailure2 = 0 Then
		intFailure2 = 1
		strLine = objTextFile.ReadLine
		strNewText = strNewText & strNewFwDate & vbCrLf
		strNewText = strNewText & strNewFwTime & vbCrLf
	Else
        strNewText = strNewText & strLine & vbCrLf
    End If
Loop
objTextFile.Close
Set objTextFile = objFSO.OpenTextFile("..\..\Common\common.h", ForWriting, True)
objTextFile.Write(strNewText)
objTextFile.Close
