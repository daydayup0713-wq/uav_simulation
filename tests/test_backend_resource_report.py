import pytest
from uav_lab_experiments.benchmark_report import resource_summary


def test_resource_summary_counts_observed_cpu_time_and_combined_peak_memory_by_phase():
    rows=[{'wall_s':0.,'phase':'replay','processes':[{'pid':1,'rss':100,'cpu_s':2.},{'pid':2,'rss':50,'cpu_s':1.}]},
          {'wall_s':2.,'phase':'replay','processes':[{'pid':1,'rss':120,'cpu_s':3.},{'pid':2,'rss':40,'cpu_s':1.5}]},
          {'wall_s':3.,'phase':'rtk_batch','processes':[{'pid':1,'rss':300,'cpu_s':3.5}]},
          {'wall_s':4.,'phase':'rtk_batch','processes':[{'pid':1,'rss':200,'cpu_s':5.}]}]
    result=resource_summary(rows)
    assert result['replay']['observed_cpu_s']==pytest.approx(1.5)
    assert result['replay']['mean_cpu_cores']==pytest.approx(.75)
    assert result['replay']['peak_combined_rss_bytes']==160
    assert result['rtk_batch']['mean_cpu_cores']==pytest.approx(1.5)
    assert result['rtk_batch']['peak_combined_rss_bytes']==300
    assert resource_summary([])=={}


def test_resource_report_rejects_clock_or_counter_regression():
    rows=[{'wall_s':t,'phase':'replay','processes':[{'pid':1,'rss':100,'cpu_s':c}]} for t,c in [(1,2),(2,1)]]
    with pytest.raises(ValueError,match='counter'):resource_summary(rows)
