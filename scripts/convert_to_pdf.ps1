$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0

$currentDir = (Get-Location).Path
[string]$docPath = [System.IO.Path]::Combine($currentDir, "Quadrotor Single Motor Failure Report.docx")
[string]$pdfPath = [System.IO.Path]::Combine($currentDir, "Quadrotor Single Motor Failure Report.pdf")

Write-Host "Opening: $docPath"
try {
    # Open(FileName, ConfirmConversions, ReadOnly, AddToRecentFiles)
    $doc = $word.Documents.Open($docPath, $false, $true, $false)
    Write-Host "Document opened. Exporting to PDF..."
    $wdExportFormatPDF = 17
    $doc.ExportAsFixedFormat($pdfPath, $wdExportFormatPDF)
    Write-Host "Export completed."
    $doc.Close($false)
    Write-Host "Successfully generated: $pdfPath"
} catch {
    Write-Error "Failed to convert docx to pdf: $_"
} finally {
    $word.Quit()
    [System.Runtime.Interopservices.Marshal]::ReleaseComObject($word) | Out-Null
    [System.GC]::Collect()
    [System.GC]::WaitForPendingFinalizers()
}
