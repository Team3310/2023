package org.frcteam2910.c2020.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frcteam2910.c2020.RobotContainer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class AutoFormatShuffleboardTab {
    private ShuffleboardTab tab = null;
    private String tabTitle = "Default";
    private int placementColIndex = 0;
    private int placementRowIndex = 0;
    private int maxHeightSetThisRow = 0;
    private int leftMostColInd = 2;
    private int rightMostColInd = 9;

    public AutoFormatShuffleboardTab(RobotContainer container) {
        this(container, "Default");
    }

    public AutoFormatShuffleboardTab(RobotContainer container, String tabTitle) {
        this(container, tabTitle, 0, 9);
    }

    public AutoFormatShuffleboardTab(RobotContainer container, String tabTitle, int leftMargin, int rightMargin) {
        this.tabTitle = tabTitle;
        tab = Shuffleboard.getTab(tabTitle);
        this.leftMostColInd = leftMargin;
        this.rightMostColInd = rightMargin;
    }

    public ComplexWidget addButtonToGrid(String title, Sendable sendable, int width, int height) {
        // By feeding 0 here, we tell the tab to add fields left-to-right, from top to bottom.
        return addButtonToGrid(title, sendable, width, height, 0);
    }

    /** This allows us to auto populate the grid, with the ability to force-start a new row. Works only with height = 1 */ 
    public ComplexWidget addButtonToGrid(String title, Sendable sendable, int width, int height, int ifAboveZeroSetRowIndexToThisValue) {
        if(tab == null) {
            // Prevent errors
            tab = Shuffleboard.getTab(tabTitle);
        }
        ComplexWidget wid = tab.add(sendable)
                                .withSize(width, height)
                                .withPosition(placementColIndex, placementRowIndex);
        if(height > maxHeightSetThisRow) {
            maxHeightSetThisRow = height;
        }
        placementColIndex += width;
        changeIndexesIfLinebreak(ifAboveZeroSetRowIndexToThisValue);
        return wid;
    }

    public SimpleWidget addToGrid(String title, Object defaultValue, int width, int height)
    {
        return addToGrid(title, defaultValue, width, height, 0);
    }

    public SimpleWidget addToGrid(String title, Object defaultValue, int width, int height, int ifAboveZeroSetRowIndexToThisValue)
    {
        if(tab == null) {
            // Prevent errors
            tab = Shuffleboard.getTab(tabTitle);
        }
        SimpleWidget wid = tab.add(title, defaultValue)
                                .withSize(width, height)
                                .withPosition(placementColIndex, placementRowIndex);
        if(height > maxHeightSetThisRow) {
            maxHeightSetThisRow = height;
        }
        placementColIndex += width;
        changeIndexesIfLinebreak(ifAboveZeroSetRowIndexToThisValue);
        return wid;
    }

    public ShuffleboardLayout addLayoutToGrid(String title, int width, int height) {
        return addLayoutToGrid(title, width, height, 0);
    }

    public ShuffleboardLayout addLayoutToGrid(String title, int width, int height, int ifAboveZeroSetRowIndexToThisValue) {
        return addLayoutToGrid(title, width, height, ifAboveZeroSetRowIndexToThisValue, BuiltInLayouts.kList);
    }

    public ShuffleboardLayout addLayoutToGrid(String title, int width, int height, BuiltInLayouts layoutType) {
        return addLayoutToGrid(title, width, height, 0, layoutType);
    }

    public ShuffleboardLayout addLayoutToGrid(String title, int width, int height, int ifAboveZeroSetRowIndexToThisValue, BuiltInLayouts layoutType) {
        if(tab == null) {
            // Prevent errors
            tab = Shuffleboard.getTab(tabTitle);
        }
        ShuffleboardLayout layout = tab.getLayout(title, layoutType)
                                        .withSize(width, height)
                                        .withPosition(placementColIndex, placementRowIndex);
        placementColIndex += width;
        changeIndexesIfLinebreak(ifAboveZeroSetRowIndexToThisValue);
        return layout;
    }

    public <T> SuppliedValueWidget<T> addNumberToGrid(String title, Supplier<T> supplier, int width, int height, int ifAboveZeroSetRowIndexToThisValue) { 
        if(tab == null) {
            tab = Shuffleboard.getTab(tabTitle);
        }
        if(supplier instanceof DoubleSupplier) {
            @SuppressWarnings("unchecked")
            SuppliedValueWidget<T> wid = (SuppliedValueWidget<T>) tab.addNumber(title, (DoubleSupplier)supplier)
                        .withSize(width, height)
                        .withPosition(placementColIndex, placementRowIndex);
            placementColIndex += width;
            changeIndexesIfLinebreak(ifAboveZeroSetRowIndexToThisValue);
            return wid;
        }
        return null;
    }
    
    public void setIndexes(int colIndex, int rowIndex) {
        placementColIndex = colIndex;
        placementRowIndex = rowIndex;
        changeIndexesIfLinebreak(0);
    }

    private void changeIndexesIfLinebreak(int ifAboveZeroSetRowIndexToThisValue) {
        if(placementColIndex >= rightMostColInd || ifAboveZeroSetRowIndexToThisValue > 0) {
            // Wrap to next row and reset col to leftmost bound
            placementColIndex = leftMostColInd;
            if(ifAboveZeroSetRowIndexToThisValue > 0)
            {
                placementRowIndex = ifAboveZeroSetRowIndexToThisValue;
            }
            else
            {
                placementColIndex += maxHeightSetThisRow;
            }
            maxHeightSetThisRow = 0;
        }
    }

    public ShuffleboardTab getShuffleboardTab() {
        return tab;
    }
}
