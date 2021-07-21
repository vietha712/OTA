using System;
using System.IO;
using System.Collections.Generic;
// The following to two namespace contains
// the functions for manipulating the
// Excel file 
using OfficeOpenXml;
using OfficeOpenXml.Style;

class Program
{
    static void Main(string[] args)
    {
#if a
        int tableRows = 10;
        int tableCols = 4;
        string Author = "MST2020";
        string Title = "Exported Data from KEPServerEX";
        string[] sheetProp = { "Sheet1", "0" };
        var excelTable = new ExcelTableForPLCData(tableRows, tableCols);

        excelTable.createExcelFile(Author, Title, sheetProp[0]);
        excelTable.initTable(Int32.Parse(sheetProp[1]));

        excelTable.appendFloatingPointData(Int32.Parse(sheetProp[1]), 3.5);
        excelTable.appendIntData(Int32.Parse(sheetProp[1]), 4);

        for(int i = 0; i < 5; i++)
        {
            excelTable.appendFloatingPointData(Int32.Parse(sheetProp[1]), i);
        }
        for (int i = 0; i < 6; i++)
        {
            excelTable.appendIntData(Int32.Parse(sheetProp[1]), i);
        }
        excelTable.appendFloatingPointData(Int32.Parse(sheetProp[1]), 5.55555);
        excelTable.appendIntData(Int32.Parse(sheetProp[1]), 123456);

        excelTable.extendTableLength(Int32.Parse(sheetProp[1]), 4);

        //Test for exception.
        //for (int i = 0; i < 20; i++)
        //{
        //    excelTable.appendFloatingPointData(0, i);
        //}
        //for (int i = 0; i < 20; i++)
        //{
        //    excelTable.appendIntData(0, i);
        //}

        excelTable.exportFile();
#endif
        /* Test for importing data */

        string path = "D:\\Workspace\\WindowApp\\ConsoleApp1\\Template.xlsx";
        var importTable = new ExcelTableForPLCData(path);
        importTable.readXLS(0); // 0 is sheet index

        for (int i = 0; i < 10; i++)
        {
            Console.Write( "Serial No. " + importTable.importData.serialNo[i].ToString() + "\n");
            Console.Write("Name. " + importTable.importData.name[i] + "\n");
            Console.Write("Value. " + importTable.importData.value[i].ToString() + "\n");
            Console.Write("Address " + importTable.importData.address[i] + "\n");
            Console.Write("Time stamp " + importTable.importData.timeStamp[i] + "\n");
            Console.Write("\n");
        }
    }

}

public class ExcelTableForPLCData
{
    public int numRows;
    public int numCols;

    private int defaultNumRows = 10;
    private int defaultNumCols = 4;
    private string[] defaultHeader = { "No.", "FloatingPointData", "intData" , "timeStamp" };
    // Default column index
    private int noCol = 1;
    private int FloatingPointDataCol = 2;
    private int IntDataCol = 3;
    private string defaultStrPath = "D:\\exportedData.xlsx";
    //Table control values
    private int currentFloatingPointDataRow = 1;
    private int currentIntDataRow = 1;
    private int numOfSheet = 0;
    private int serialNumber = 1; //init value
    private int maxRows = 3; // less than this value, more row can be appended
    private ExcelPackage excelFile;
    public templateForImport importData;

    public struct templateForImport
    {
        public int[] serialNo;
        public string[] name;
        public double[] value;
        public int[] address;
        public string[] timeStamp;
    }

    public ExcelTableForPLCData()
    {
        this.numRows = defaultNumRows;
        this.numCols = defaultNumCols;
        currentFloatingPointDataRow = 1;
        currentIntDataRow = 1;
        excelFile = new ExcelPackage();
    }
    public ExcelTableForPLCData(int rows, int cols)
    {
        this.numRows = rows;
        this.numCols = cols;
        currentFloatingPointDataRow = 1;
        currentIntDataRow = 1;
        excelFile = new ExcelPackage();
    }

    public ExcelTableForPLCData(string filePath)
    {
        FileInfo fileInfo = new FileInfo(filePath);
        excelFile = new ExcelPackage(fileInfo);

        excelFile.Save();
    }

    public void createExcelFile(string Author, string Title, string sheetName)
    {
        excelFile.Workbook.Properties.Author = Author;
        excelFile.Workbook.Properties.Title = Title;
        excelFile.Workbook.Worksheets.Add(sheetName);
        numOfSheet++;

        excelFile.Save();
    }

    public void addSheet(string sheetName)
    {
        excelFile.Workbook.Worksheets.Add(sheetName);
        numOfSheet++;
        excelFile.Save();
    }

    public void exportFile()
    {
        if (File.Exists(defaultStrPath))
            File.Delete(defaultStrPath);

        // Create excel file on physical disk 
        FileStream objFileStrm = File.Create(defaultStrPath);
        objFileStrm.Close();

        // Write content to excel file 
        File.WriteAllBytes(defaultStrPath, excelFile.GetAsByteArray());
        //Close Excel package
        excelFile.Dispose();
    }

    public void initTable(int sheetIdx)
    {

        excelFile.Workbook.Worksheets[sheetIdx].TabColor = System.Drawing.Color.Black;
        excelFile.Workbook.Worksheets[sheetIdx].DefaultRowHeight = 12;

        // Setting the properties of the first row (header)
        excelFile.Workbook.Worksheets[sheetIdx].Row(1).Height = 20;
        excelFile.Workbook.Worksheets[sheetIdx].Row(1).Style.HorizontalAlignment = ExcelHorizontalAlignment.Center;
        excelFile.Workbook.Worksheets[sheetIdx].Row(1).Style.Font.Bold = true;
        // Header of the Excel sheet
        for(int i = 1; i <= numCols; i++)
        {
            excelFile.Workbook.Worksheets[sheetIdx].Cells[1, i].Value = defaultHeader[i-1];
            excelFile.Workbook.Worksheets[sheetIdx].Column(i).AutoFit();
        }
        // Fill up serial num
        this.extendTableLength(sheetIdx, numRows);

        currentFloatingPointDataRow++;
        currentIntDataRow++;

        excelFile.Save();
    }

    public void appendFloatingPointData(int sheetIdx, double inData)
    {
        if (currentFloatingPointDataRow >= maxRows)
        {
            throw new InvalidOperationException("Cannot append over the length of the table");
        }
        excelFile.Workbook.Worksheets[sheetIdx].Cells[currentFloatingPointDataRow, FloatingPointDataCol].Value = inData; 
        excelFile.Save();
        currentFloatingPointDataRow++;
    }

    public void appendIntData(int sheetIdx, int inData)
    {
        if (currentIntDataRow >= maxRows )
        {
            throw new InvalidOperationException("Cannot append over the length of the table");
        }
        excelFile.Workbook.Worksheets[sheetIdx].Cells[currentIntDataRow, IntDataCol].Value = inData;
        excelFile.Save();
        currentIntDataRow++;
    }

    public void extendTableLength(int sheetIdx, int numRows)
    {
        for(int i = 1; i <= numRows; serialNumber++, i++)
        {
            excelFile.Workbook.Worksheets[sheetIdx].Cells[serialNumber+1, noCol].Value = serialNumber;
            excelFile.Workbook.Worksheets[sheetIdx].Row(serialNumber+1).Style.HorizontalAlignment = ExcelHorizontalAlignment.Center;
        }
        maxRows = serialNumber + 1;

        excelFile.Save();
    }

    public void readXLS(int sheetIdx)
    {
        //get the first worksheet in the workbook
        ExcelWorksheet worksheet = excelFile.Workbook.Worksheets[sheetIdx];
        int colCount = worksheet.Dimension.End.Column;  //get Column Count
        int rowCount = worksheet.Dimension.End.Row;     //get row count
        importData = new templateForImport();
        importData.serialNo = new int[rowCount];
        importData.name = new string[rowCount];
        importData.value = new double[rowCount];
        importData.address = new int[rowCount];
        importData.timeStamp = new string[rowCount];

        for(int i = 2; i <= rowCount; i++)
        {
            importData.serialNo[i - 2] = Int32.Parse(worksheet.Cells[i, 1].Value.ToString().Trim());
            importData.name[i - 2] = worksheet.Cells[i, 2].Value.ToString().Trim();
            importData.value[i - 2] = Double.Parse(worksheet.Cells[i, 3].Value.ToString().Trim());
            importData.address[i - 2] = Int32.Parse(worksheet.Cells[i, 4].Value.ToString().Trim());
            importData.timeStamp[i - 2] = worksheet.Cells[i, 5].Value.ToString().Trim();
        }
    }
}

/* Line chart for the variation of the PLC data */
/* To be implemented: Improting the excel file data to PLC data */








