# PID Controller implementation
# https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html

import streamlit as st
import altair as alt
import numpy as np
import pandas as pd


def proportional(kp, sp):
    """Creates a proportional controller with a specified gain"""
    mv = 0
    while True:
        pv = yield mv
        mv = kp * (sp - pv)


def pid(kp, ki, kd, mvbar=0):
    """Creates a PID controller with specified control constants"""

    e_prev = 0
    t_prev = -1
    i = 0

    mv = mvbar

    while True:
        # yield mv, wait for new t, pv, sp
        t, pv, sp = yield mv

        # PID Calculations
        e = sp - pv

        p = kp * e
        i = i + ki * e * (t - t_prev)
        d = kd * (e - e_prev) / (t - t_prev)

        mv = mvbar + p + i + d

        # update stored data for next run
        e_prev = e
        t_prev = t


code = '''def pid(kp, ki, kd, mvbar=0):
    """
    Creates a PID controller with control constants
    ref: 
    https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html"""
    e_prev = 0
    t_prev = -1
    i = 0
    mv = mvbar
    while True:
        # yield mv, wait for new t, pv, sp
        t, pv, sp = yield mv
        # PID Calculations
        e = sp - pv
        p = kp * e
        i = i + ki * e * (t - t_prev)
        d = kd * (e - e_prev) / (t - t_prev)
        mv = mvbar + p + i + d
        # update stored data for next run
        e_prev = e
        t_prev = t
'''

st.title("Simple PID implementations")
with st.expander("Code:"):
    st.code(code, language="python")

st.header("Step signal")
sig = st.number_input("Target value after step", value=10)
tmx = st.number_input("Tmax: length of period", value=1000)
col1, col2, col3 = st.columns(3)
# kp = col1.number_input("Kp Proportional", value=0.1)
# ki = col2.number_input("Ki Integral", value=0.1)
# kd = col3.number_input("Kd Derivative", value=0.1)

kp = col1.slider("Kp Proportional", min_value=0.0, max_value=1.0, value=0.03)
ki = col2.slider("Ki Integral", min_value=0.0, max_value=1.0, value=0.03)
kd = col3.slider("Kd Derivative", min_value=0.0, max_value=1.0, value=0.03)

con1 = pid(kp, ki, kd)
con1.send(None)


df_step = pd.DataFrame({"t": np.linspace(0, tmx, int(tmx))})
df_step["sp"] = np.where(df_step["t"] > tmx / 2, sig, 0)
pv = [0]
mv = [0]

for i, x in enumerate(list(df_step["t"])):
    if i == 0:
        continue
    mv.append(con1.send([x, pv[i - 1], df_step.iloc[i]["sp"]]))
    pv.append(mv[i] + pv[i - 1])

df_step["mv"] = mv
df_step["pv"] = pv

dfm_step = pd.melt(df_step, id_vars="t", value_vars=["pv", "sp"])

stepsignal = (
    alt.Chart(dfm_step)
    .mark_line()
    .encode(
        x=alt.X(
            "t",
            axis=alt.Axis(title="Time"),
        ),
        y=alt.Y("value", axis=alt.Axis(title="Signal")),
        color="variable",
    )
)

st.altair_chart(stepsignal, use_container_width=True)

st.header("Perturbed step signal")
con2 = pid(kp, ki, kd)
con2.send(None)

df_rand = pd.DataFrame({"t": np.linspace(0, tmx, int(tmx))})
df_rand["sp"] = np.where(df_rand["t"] > tmx / 2, sig, 0)
pv = [0]
mv = [0]

for i, x in enumerate(list(df_rand["t"])):
    if i == 0:
        continue
    mv.append(con2.send([x, pv[i - 1], df_rand.iloc[i]["sp"]]))
    pv.append(mv[i] + pv[i - 1] + (np.random.rand() - 0.5) * sig / 4)

df_rand["mv"] = mv
df_rand["pv"] = pv

dfm_rand = pd.melt(df_rand, id_vars="t", value_vars=["pv", "sp"])

steprand = (
    alt.Chart(dfm_rand)
    .mark_line()
    .encode(
        x=alt.X(
            "t",
            axis=alt.Axis(title="Time"),
        ),
        y=alt.Y("value", axis=alt.Axis(title="Signal")),
        color="variable",
    )
)

st.altair_chart(steprand, use_container_width=True)
