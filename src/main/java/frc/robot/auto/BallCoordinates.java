package frc.robot.auto;

public class BallCoordinates {
    private String m_label = "default";
    private int m_ymin;
    private int m_xmin;
    private int m_ymax;
    private int m_xmax;
    private double m_confidence;

    public String getLabel() {
        return m_label;
    }

    public int getYMin() {
        return m_ymin;
    }

    public int getXMin() {
        return m_xmin;
    }

    public int getYMax() {
        return m_ymax;
    }

    public int getXMax() {
        return m_xmax;
    }

    public double getConfidence() {
        return m_confidence;
    }

    public void setLabel(final String label) {
        m_label = label;
    }

    public void setYMin(final int ymin) {
        m_ymin = ymin;
    }

    public void setXMin(final int xmin) {
        m_xmin = xmin;
    }

    public void setYMax(final int ymax) {
        m_ymax = ymax;
    }

    public void setXMax(final int xmax) {
        m_xmax = xmax;
    }

    public void setConfidence(final double confidence) {
        m_confidence = confidence;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("label = " + m_label + ", ");
        sb.append("ymin = " + m_ymin + ", ");
        sb.append("xmin = " + m_xmin + ", ");
        sb.append("ymax = " + m_ymax + ", ");
        sb.append("xmax = " + m_xmax + ", ");
        sb.append("confidence = " + m_confidence + ".");
        return sb.toString();
    }
} 